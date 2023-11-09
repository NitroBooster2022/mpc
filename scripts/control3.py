#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import String, Byte
from utils.msg import Lane, Sign, localisation, IMU, encoder
# from utils.srv import get_direction, dotted, nav
import time
import math

import cv2
import os
import json
import threading
import argparse
import socket

import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from trackmap import track_map

class StateMachine():
    #initialization
    def __init__(self, simulation = False, planned_path = "/paths/path.json", custom_path = False):
        rospy.init_node('control_node', anonymous=True)
        self.rate = rospy.Rate(25)
        self.dt = 1/25 #for PID

        #simulation
        self.simulation = simulation
        if self.simulation:
            print("Simulation mode")
            self.publish_cmd_vel = self.publish_cmd_vel_sim
            self.cmd_vel_pub = rospy.Publisher("/automobile/command", String, queue_size=3)
            self.min_sizes = [25,25,40,50,45,35,30,25,25,130,75,72,125]
            self.max_sizes = [100,75,125,100,120,125,70,75,100,350,170,250,320]
            self.odomRatio = 1
            self.process_yaw = self.process_yaw_sim
            self.left_trajectory = self.left_trajectory_sim
            self.right_trajectory = self.right_trajectory_sim
            self.right_exit_trajectory = self.right_exit_trajectory_sim
            self.left_exit_trajectory = self.left_exit_trajectory_sim
            self.parallelParkAngle = 45
            self.overtakeAngle = np.pi/5
            self.initializationTime = 2
            self.maxspeed = 0.175
            file = open(os.path.dirname(os.path.realpath(__file__))+'/PIDSim.json', 'r')
            if custom_path:
                self.track_map = track_map()
                self.track_map.custum_path()
            self.plan_path(custom_path, planned_path)
        else:
            self._init_socket_semaphore()
            # serialNODE
            from messageconverter import MessageConverter
            import serial
            devFile = '/dev/ttyACM0'
            self.publish_cmd_vel = self.publish_cmd_vel_real
            self.min_sizes = [25,25,40,50,45,35,30,25,25,150,75,72,125]
            self.max_sizes = [100,75,125,100,120,125,70,75,100,350,170,250,300]
            # comm init
            self.serialCom = serial.Serial(devFile,19200,timeout=1)
            self.serialCom.flushInput()
            self.serialCom.flushOutput()
            
            # message converted init
            self.messageConverter = MessageConverter()
            
            # get initial yaw from IMU
            self.initialYaw = 0
            #launch sensors at 0 to remove this
            #or get the yaw offset from 0 after run
            while self.initialYaw==0:
                imu = rospy.wait_for_message("/automobile/IMU",IMU)
                self.initialYaw = imu.yaw
                print("initialYaw: "+str(self.initialYaw))
            print("Real mode")
            self.odomRatio = 0.0066
            self.process_yaw = self.process_yaw_real
            self.left_trajectory = self.left_trajectory_real
            self.right_trajectory = self.right_trajectory_real
            self.right_exit_trajectory = self.right_exit_trajectory_real
            self.left_exit_trajectory = self.left_exit_trajectory_real
            self.parallelParkAngle = 35
            self.overtakeAngle = np.pi/5
            self.initializationTime = 3.57
            self.maxspeed = 0.15
            file = open(os.path.dirname(os.path.realpath(__file__))+'/PID.json', 'r')
            
            self.plan_path(custom_path, planned_path)
            
            #0:left, 1:straight, 2:right, 3:parkF, 4:parkP, 5:exitparkL, 6:exitparkR, 7:exitparkP
            #8:enterhwLeft, 9:enterhwStright, 10:rdb, 11:exitrdbE, 12:exitrdbS, 13:exitrdbW, 14:curvedpath
            # self.decisions = [2,3,6,0,4]
            # self.decisions = [3,5,1]
            # self.decisions = [2,2,2,2,2,2,2,2,2]
            # self.decisions = [10,12]
            # self.decisionsI = 0
            # self.full_path = ['test','test','test','test','test','test','test','test','test','test','test','test','test']
            # self.planned_path = ['test1']
        #states
        self.states = ['Lane Following', "Approaching Intersection", "Stopping at Intersection", 
                       "Intersection Maneuvering", "Approaching Crosswalk", "Pedestrian", "Highway",
                       "overtaking", "Roundabout", "Parking", "Initial", "Parked", "Curvedpath"] #13 states
        self.state = 10 #initial
        self.history = 0
        self.highwaySpeed = self.maxspeed*1.33
        self.highwayRight = True
        self.highwaySide = 1 #1:right, -1:left
        self.laneOvertakeAngle = np.pi*0.175
        self.laneOvertakeCD = 2
        self.overtakeDuration = 1
        self.roadblock = True

        #sign
        self.class_names = ['oneway', 'highwayentrance', 'stopsign', 'roundabout', 'park', 'crosswalk', 'noentry', 'highwayexit', 'priority',
                'lights','block','pedestrian','car','others','nothing']
        # self.min_sizes = [25,25,40,50,40,35,30,25,25,130,75,72,130]
        # self.max_sizes = [100,75,125,100,120,125,70,75,100,350,170,250,300]
        self.center = -1
        self.image_center = 320
        self.ArrivedAtStopline = False
        self.detected_objects = []
        self.numObj = -1
        self.box1 = []
        self.box2 = []
        self.box3 = []
        self.confidence = []

        #pose (will get them from localisation)
        self.x = 0.82
        self.y = 14.91
        self.yaw = 1.5707
        self.speed = 0
        self.odomX = 0
        self.odomY = 0

        #PID
        #for initial angle adjustment
        self.error_sum = 0
        self.last_error = 0
        #for goal points
        self.error_sum2 = 0
        self.last_error2 = 0
        # Load PIDs data
        data = json.load(file)
        print("PIDs params:")
        print(data)
        self.p = data.get('p')
        self.d = data.get('d')
        self.i = data.get('i')
        self.kp = data.get('kp')
        self.kd = data.get('kd')
        self.ki = data.get('ki')
        self.kp2 = data.get('kp2')
        self.kd2 = data.get('kd2')
        self.ki2 = data.get('ki2')

        #steering
        self.msg = String()
        self.msg2 = String()
        self.last = 0
        self.pl = 320 # previous lane center

        #timers
        self.timer = None
        self.timerpid = rospy.Time.now()
        self.timerodom = rospy.Time.now()
        self.timerPedestrian = None
        self.timerP = None
        self.timerO = None
        self.timer1 = None
        self.timer2 = None
        self.exitCD = rospy.Time.now()
        self.flag1 = True

        #intersection & parking
        #0:left, 1:straight, 2:right, 3:parkF, 4:parkP, 5:exitparkL, 6:exitparkR, 7:exitparkP
        #8:enterhwLeft, 9:enterhwStright, 10:rdb, 11:exitrdbE, 12:exitrdbS, 13:exitrdbW, 14:curvedpath
        self.intersectionStop = False
        self.intersectionDecision = -1
        self.parkingDecision = -1
        self.exitDecision = -1
        self.rdbDecision = -1
        self.decisionList = ["left","straight","right","parkF","parkP",
        "exitparkL","exitparkR","exitparkP","enterhwLeft","enterhwStright","rdb",
        "exitrdbE","exitrdbS","exitrdbW","curvedpath"]
        self.doneManeuvering = False
        self.doneParking = False
        self.initialPoints = None
        self.intersectionState = 0
        self.trajectory = None
        #constants
        self.orientation = 1 #0,1,2,3=east,north,west,south
        self.directions = ["east", "north", "west", "south"]
        self.orientations = np.array([0,1.5708,3.14159,4.7124]) #0, pi/2, pi, 3pi/2
        self.left_offset_x = 1.20
        self.left_offset_y = 0.82
        self.right_offset_x = 0.80
        self.right_offset_y = 0.573
        self.velocity = self.maxspeed
        self.offsets_x = np.array([self.left_offset_x, self.left_offset_x*1.2, self.right_offset_x])
        self.offsets_y = np.array([self.left_offset_y, 0, self.right_offset_y])
        self.rotation_matrices = np.array([[[1,0],[0,1]],[[0,-1],[1,0]],[[-1,0],[0,-1]],[[0,1],[-1,0]]]) #E,N,W,S
        self.offset = 0.3

        # Create service proxy
        # self.get_dir = rospy.ServiceProxy('get_direction',get_direction)
        # self.get_dotted = rospy.ServiceProxy('dotted',dotted)
        # rospy.wait_for_service('dotted')
        # how to use:
        # d = self.get_dotted("dotted").dotted
        print("hello")
        # Subscribe to topics
        self.lane_sub = rospy.Subscriber('lane', Lane, self.lane_callback, queue_size=3)
        self.sign_sub = rospy.Subscriber('sign', Sign, self.sign_callback, queue_size=3)
        # self.localization_sub = message_filters.Subscriber("/automobile/localisation", localisation, queue_size=3)
        self.imu_sub = rospy.Subscriber("/automobile/IMU", IMU, self.imu_callback, queue_size=3)
        self.encoder_sub = rospy.Subscriber("/automobile/encoder", encoder, self.encoder_callback, queue_size=3)

        print("hello1")
        #stop at shutdown
        def shutdown():
            self.lane_sub.unregister()
            if self.simulation:
                pub = rospy.Publisher("/automobile/command", String, queue_size=3)
                msg = String()
                msg2 = String()
                # msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
                msg.data = '{"action":"1","speed":'+str(0.0)+'}'
                msg2.data = '{"action":"2","steerAngle":'+str(0.0)+'}'
                for haha in range(10):
                    pub.publish(msg2)
                    pub.publish(msg)
                    self.rate.sleep()
            else:
                msg = String()
                msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
                for haha in range(20):
                    self._write(msg)
                    self.rate.sleep()
        
        rospy.on_shutdown(shutdown)

        self.parkAdjust = True #adjust flag for forward/backward

        #size of detected objects
        self.parksize = 0
        self.parkSecond = False
        self.carsize = 0

        #flag for light, highway, curvedpath and roadblock
        self.light = False
        self.hw = False
        self.cp = False
        self.roadblock = False

        self.rdbExitYaw = 0
        # self.rdbTransf = 0
        # self.carBlockSem = -1
        self.toggle = 0
        # self.t1 = time.time()
        self.adjustYawError = 0.2 #yaw adjust for intersection maneuvering
        self.localise_before_decision = False
    
    def _init_socket_semaphore(self):
        # Communication parameters, create and bind socket
        self.PORT = 50007
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) #(internet, UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        self.sock.bind(('', self.PORT))
        self.sock.settimeout(1)

    def localise(self):
        try:
            imu = rospy.wait_for_message("/automobile/IMU",IMU)
            self.yaw = imu.yaw
            loc = rospy.wait_for_message("/automobile/localisation",localisation,timeout=3)
            self.x = loc.posA
            self.y = loc.posB
        except:
            print("localisation timed out")
            self.idle()
            self.idle()
            self.idle()
            rospy.signal_shutdown("Exit")
        print("x,y,yaw",self.x,self.y,self.yaw)

    def plan_path(self, custom_path, planned_path):
        self.localise()
        if custom_path:
            self.track_map.location = self.track_map.locate(self.x,self.y,self.yaw)
            self.track_map.plan_path()
            self.planned_path = self.track_map.planned_path
        else:
            self.planned_path = json.load(open(os.path.dirname(os.path.realpath(__file__))+planned_path, 'r'))
            self.track_map = track_map(self.x,self.y,self.yaw,self.planned_path)
            self.track_map.plan_path()
        if self.track_map.location == "highwayN" or self.track_map.location == "highwayS":
            self.hw = True
            self.history = 6
        elif self.track_map.location == "curvedpath":
            self.cp = True
        # self.track_map.draw_map()
        #0:left, 1:straight, 2:right, 3:parkF, 4:parkP, 5:exitparkL, 6:exitparkR, 7:exitparkP
        #8:enterHWLeft, 9:enterHWStraight, 10:rdb, 11:exitrdbE, 12:exitrdbS, 13:exitrdbW, 14:curvedpath
        self.decisions = self.track_map.directions
        self.decisionsI = 0
        self.full_path = self.track_map.path
        if self.full_path[self.decisionsI] == self.planned_path[0]:
            self.planned_path.pop(0)
        i = 0
        while i < len(self.planned_path) - 1:
            if self.planned_path[i] == "track1N" and self.planned_path[i+1] == "track2N":
                self.planned_path.pop(i)
            elif self.planned_path[i] == "track2S" and self.planned_path[i+1] == "track1S":
                self.planned_path.pop(i)
            else:
                i += 1
        i = 0
        while i < len(self.full_path) - 1:
            if self.full_path[i] == "track1N" and self.full_path[i+1] == "track2N":
                self.full_path.pop(i)
            elif self.full_path[i] == "track2S" and self.full_path[i+1] == "track1S":
                self.full_path.pop(i)
            else:
                i += 1
        print("Planned path: ", self.planned_path)
        print("Full path: ", self.full_path)
    
    def plan_new_path(self):
        self.localise()
        self.track_map.location = self.track_map.locate(self.x,self.y,self.yaw)
        self.track_map.planned_path = self.planned_path
        self.track_map.plan_path()
        if self.track_map.location == "highwayN" or self.track_map.location == "highwayS":
            self.hw = True
            self.history = 6
        elif self.track_map.location == "curvedpath":
            self.cp = True
        # self.track_map.draw_map()
        #0:left, 1:straight, 2:right, 3:parkF, 4:parkP, 5:exitparkL, 6:exitparkR, 7:exitparkP
        #8:enterHWLeft, 9:enterHWStraight, 10:rdb, 11:exitrdbE, 12:exitrdbS, 13:exitrdbW, 14:curvedpath
        self.decisions = self.track_map.directions
        self.decisionsI = 0
        self.full_path = self.track_map.path
        if self.full_path[self.decisionsI] == self.planned_path[0]:
            self.planned_path.pop(0)
        print("Planned path: ", self.planned_path)
        print("Full path: ", self.full_path)
        i = 0
        while i < len(self.planned_path) - 1:
            if self.planned_path[i] == "track1N" and self.planned_path[i+1] == "track2N":
                self.planned_path.pop(i)
            elif self.planned_path[i] == "track2S" and self.planned_path[i+1] == "track1S":
                self.planned_path.pop(i)
            else:
                i += 1
        i = 0
        while i < len(self.full_path) - 1:
            if self.full_path[i] == "track1N" and self.full_path[i+1] == "track2N":
                self.full_path.pop(i)
            elif self.full_path[i] == "track2S" and self.full_path[i+1] == "track1S":
                self.full_path.pop(i)
            else:
                i += 1

    def _write(self, msg):
        """ Represents the writing activity on the the serial.
        """
        # print(msg.data)
        command = json.loads(msg.data)
        command_msg = self.messageConverter.get_command(**command)
        # print(command_msg)
        self.serialCom.write(command_msg.encode('ascii'))
        # buffer_size = self.serialCom.out_waiting
        # print("Buffer size:", buffer_size)

    def process_yaw_sim(self, yaw):
        self.yaw = yaw if yaw>0 else (6.2831853+yaw)
    def process_yaw_real(self, yaw):
        if yaw!=0:
            newYaw = -((yaw-self.initialYaw)*3.14159/180)
            self.yaw = newYaw if newYaw>0 else (6.2831853+newYaw)

    #callback functions
    def lane_callback(self,lane):
        self.center = lane.center
        self.ArrivedAtStopline = lane.stopline
        # if there's a big shift in lane center: ignore due to delay
        if abs(self.center-self.pl)>250:
            self.center = self.pl
        # ignore one center measurement when we don't detect
        if self.center==320:
            c = self.center
            self.center = self.pl
            self.pl = c
        else:
            self.pl = self.center
        # print("time: ", time.time()-self.t1)
        # self.t1 = time.time()
        # print("action")
        act = self.action()
        if int(act)==1:
            print(f"-----transitioning to '{self.states[self.state]}'-----")
            if self.state==0:
                print("Speed is at "+str(self.maxspeed)+"m/s")
    def sign_callback(self,sign):
        self.detected_objects = sign.objects
        self.numObj = sign.num
        self.box1 = sign.box1
        self.box2 = sign.box2
        self.box3 = sign.box3
        self.box4 = sign.box4
        self.confidence = sign.confidence
    def encoder_callback(self,encoder):
        self.velocity = encoder.speed
    def imu_callback(self,imu):
        self.process_yaw(imu.yaw)

    #state machine
    def action(self):
        if self.state==0: #lane following
            return self.lanefollow()
        elif self.state == 1: #Approaching Intersection
            return self.approachInt()
        elif self.state == 2: #Stopping at Intersection
            return self.stopInt()
        elif self.state == 3: #Intersection Maneuvering
            return self.maneuverInt()
        elif self.state == 4: #Approaching Crosswalk
            return self.approachCrosswalk()
        elif self.state == 5: #Pedestrian
            return self.stopPedestrian()
        elif self.state == 6: #Highway
            return self.highway()
        elif self.state == 7: #overtake
            if self.history == 6:
                return self.overtake()
            else:
                return self.lane_overtake()
        elif self.state == 8: #Roundabout
            return self.roundabout()
        elif self.state == 9: #Parking
            if self.timerP is None:
                self.timerP = rospy.Time.now() + rospy.Duration(1.57) # stop before parking
                print("prepare to park")
            elif rospy.Time.now() >= self.timerP:
                return self.park()
            self.idle()
            return 0
        elif self.state == 10: #initialization state
            # self.localise()
            if self.timer is None:
                print("Initializing controller...")
                self.timer = rospy.Time.now() + rospy.Duration(self.initializationTime)
            if rospy.Time.now() >= self.timer:
                print("done initializing.")
                self.timer = None
                self.state = self.history
                return 1
            else:
                if self.toggle == 0:
                    self.toggle = 1
                    if self.simulation:
                        self.idle()
                    else:
                        self.msg.data = '{"action":"4","activate": true}'
                elif self.toggle == 1: 
                    self.toggle = 2
                    self.idle()
                elif self.toggle == 2:
                    self.toggle = 0
                    if self.simulation:
                        self.idle()
                    else:
                        self.msg.data = '{"action":"5","activate": true}'
                if not self.simulation:
                    self._write(self.msg)
                return 0
        elif self.state == 11: #parked
            # if self.decisionsI >= len(self.decisions):
            #     self.idle()
            #     self.idle()
            #     self.idle()
            #     rospy.signal_shutdown("Exit")
            #     return 0
            # else:
                if self.timerP is None:
                    self.timerP = rospy.Time.now() + rospy.Duration(1.57) # stop before parking
                    print("prepare to exit")
                elif rospy.Time.now() >= self.timerP:
                    return self.exitPark()
                self.idle()
                return 0
        elif self.state == 12: #Curvedpath
            return self.curvedpath()
    
    #actions
    def lanefollow(self):
        #transition events
        if self.stop_sign_detected() and self.timer2 is None:
            print("stop sign detected -> state 1")
            self.intersectionStop = True
            self.state = 1
            self.timer0 = None
            self.timer2 = None
            return 1
        elif self.light_detected() and self.timer2 is None:
            if self.is_green():
                print("green light detected -> state 1")
                self.intersectionStop = False
            else:
                print("red light detected -> state 1")
                self.intersectionStop = True
                self.light = True
            self.state = 1
            self.timer2 = None
            self.timer0 = None
            return 1
        elif self.crosswalk_sign_detected() and self.timer2 is None:
            print("crosswalk sign detected -> state 4")
            self.state = 4
            return 1
        elif self.pedestrian_appears():
            print("pedestrian appears!!! -> state 5")
            self.history = self.state
            self.state = 5
            self.timerPedestrian = rospy.Time.now()+rospy.Duration(2.5)
            return 1
        car_sizes =  self.get_car_size(minSize = 100)
        num = len(car_sizes)
        rightCar = False
        for box in car_sizes:
            if box[0] + box[2]/2 > 144:
                rightCar = True
                break
        if num > 0 and rightCar and rospy.Time.now() > self.exitCD:
            if self.timerO is None:
                print("car detected, waiting for 2s to ascertain car's speed")
                self.timerO = rospy.Time.now() + rospy.Duration(2)
                self.highwaySide = 1
                self.numCars, self.firstDetectionSizes = num, car_sizes
            if rospy.Time.now() >= self.timerO:
                if num < 2:
                    print("only one car, overtake, side is ", self.highwaySide) 
                    self.timerO = None
                    self.history = self.state
                    self.overtakeAngle = np.pi*0.3
                    self.state = 7 #overtake
                    return 1
                self.idle()
                return 0
            else:
                self.publish_cmd_vel(0,0)
                return 0
        if self.timer2 is not None and rospy.Time.now() >= self.timer2 and self.highwaySide == -1:
            #go back to right side
            print("timer2 expired & no car in sight. going back to right side")
            self.timerO = None
            self.timer2 = None
            self.history = self.state
            self.overtakeAngle = np.pi*0.2
            self.state = 7 #switch lane
            return 1
        
        roadblock_sizes =  self.get_obj_size(obj_id = 10)
        num = len(roadblock_sizes)
        rightBlock = False
        for box in roadblock_sizes:
            if box[0] + box[2]/2 > 144:
                rightBlock = True
                break
        if num > 0 and rightBlock:
            if self.timerO is None:
                print("roadblock detected, waiting for 2s to ascertain position")
                self.timerO = rospy.Time.now() + rospy.Duration(2)
                self.highwaySide = 1
            if rospy.Time.now() >= self.timerO and num < 2:
                print("overtake roadblock!") 
                self.timerO = None
                self.history = self.state
                self.overtakeAngle = np.pi*0.3
                self.roadblock = True
                self.state = 7 #overtake
                return 1
            else:
                self.publish_cmd_vel(0,0)
                return 0
        elif self.parking_detected():
            # if not at parking decision yet pass
            if self.decisionsI >= len(self.decisions):
                self.publish_cmd_vel(self.get_steering_angle())
                return 0
            elif (self.decisions[self.decisionsI] != 3 and self.decisions[self.decisionsI] != 4):
                self.publish_cmd_vel(self.get_steering_angle())
                return 0
            if self.detected_objects[0] == 4:
                self.parksize = max(self.box1[2], self.box1[3])
                try:
                    if self.detected_objects[1] == 12:
                        self.carsize = max(self.box2[2], self.box2[3])
                except:
                    self.carsize = 0
            else:
                self.parksize = max(self.box2[2], self.box2[3])
                if self.detected_objects[0] == 12:
                    self.carsize = max(self.box1[2], self.box1[3])
            self.parksize = self.parksize*0.00263
            print("about to park -> 9")
            self.timer0 = None
            self.state = 9
            return 1
        elif self.ArrivedAtStopline:
            print("signless intersection detected... -> state 3")
            self.doneManeuvering = False #set to false before entering state 3
            self.state = 3
            self.timer0 = None
            self.timer2 = None
            return 1
        # Determine the steering angle based on the center and publish the steering command
        self.publish_cmd_vel(self.get_steering_angle())
        return 0
    
    def approachInt(self):
        #Transition events
        if self.ArrivedAtStopline:
            if self.intersectionStop:
                print("arrived at stopline. Transitioning to 'stopping at intersection'.")
                self.state = 2
                return 1
            else:
                print("arrived at stopline. Transitioning to 'intersection maneuvering' directly.")
                self.doneManeuvering = False #set to false before entering state 3
                self.state = 3
                return 1
        elif self.pedestrian_appears():
            print("pedestrian appears!!! -> state 5")
            self.history = self.state
            self.state = 5
            self.timerPedestrian = rospy.Time.now()+rospy.Duration(2.5)
            return 1
        # Determine the steering angle based on the center publish the steering command
        self.publish_cmd_vel(self.get_steering_angle()) 
        return 0
    
    def stopInt(self):
        #Transition events
        if self.timer is None:
            self.timer = rospy.Time.now() + rospy.Duration(3.57)
        elif rospy.Time.now() >= self.timer:
            self.timer = None
            self.doneManeuvering = False #set to false before entering state 3
            self.state = 3
            return 1
        elif self.light:
            #call service to check light color
            if self.is_green():
                # print("green")
                self.light = False
                self.timer = None
                self.doneManeuvering = False #set to false before entering state 3
                self.state = 3
                return 1
            else:
                # print("red")
                self.timer = rospy.Time.now() + rospy.Duration(3.57)
        self.idle()
        return 0
    
    def maneuverInt(self):
        # need to reposition by changing trajectories
        # if self.roadblock_detected() and not self.roadblock and (abs(self.yaw-self.destinationAngle) <= 0.25 or abs(self.yaw-self.destinationAngle) >= 6.03):
        #     self.roadblock = True
        #     print("roadblock detected: recalculate path")
        #     if self.simulation:
        #         dests = self.track_map.get_location_dest(self.full_path[self.decisionsI])
        #     else:
        #         dests = [0,1] #CHANGE
        #     if self.intersectionDecision == 0:
        #         if 1 in dests:
        #             self.destinationAngle -= np.pi/2
        #             if self.destinationAngle < -0.5:
        #                 self.destinationAngle += np.pi*2
        #             self.trajectory = self.straight_trajectory
        #             self.intersectionDecision = 1
        #             self.kd2 += 5
        #         else:
        #             self.destinationAngle -= np.pi
        #             if self.destinationAngle < -0.5:
        #                 self.destinationAngle += np.pi*2
        #             self.trajectory = self.right_trajectory
        #             self.intersectionDecision = 2
        #     elif self.intersectionDecision == 1:
        #         if 0 in dests:
        #             self.destinationAngle += np.pi/2
        #             if self.destinationAngle > 5.5:
        #                 self.destinationAngle = 0
        #             self.trajectory = self.left_trajectory
        #             self.intersectionDecision = 0
        #         else:
        #             self.destinationAngle -= np.pi/2
        #             if self.destinationAngle < 0.5:
        #                 self.destinationAngle += np.pi*2
        #             self.trajectory = self.right_trajectory
        #             self.intersectionDecision = 2
        #     else:
        #         if 1 in dests:
        #             self.destinationAngle += np.pi/2
        #             if self.destinationAngle > 5.5:
        #                 self.destinationAngle = 0
        #             self.trajectory = self.straight_trajectory
        #             self.intersectionDecision = 1
        #             self.kd2 += 5
        #         else:
        #             self.destinationAngle -= np.pi
        #             if self.destinationAngle < -0.5:
        #                 self.destinationAngle += np.pi*2
        #             self.trajectory = self.left_trajectory
        #             self.intersectionDecision = 0
        #     return 0

        if self.pedestrian_appears():
            print("pedestrian appears!!! -> state 5")
            self.history = self.state
            self.state = 5
            self.timerPedestrian = rospy.Time.now()+rospy.Duration(2.5)
            return 1
        if self.doneManeuvering:
            print("done intersection maneuvering.")
            self.doneManeuvering = False #reset
            self.intersectionDecision = -1 #reset
            if self.hw:
                print("entering highway -> 6")
                self.state = 6
            elif self.cp:
                self.state = 8
            else:
                self.state = 0 #go back to lane following
            self.hw = False
            self.cp = False
            if self.roadblock:
                self.roadblock = False
                self.kd2 = 1
                print("done new path after roadblock")
            self.initialPoints = None #reset initial points
            self.pl = 320
            self.adjustYawError = 0.2
            return 1
        elif self.intersectionDecision < 0:
            if self.decisionsI >= len(self.decisions):
                self.idle()
                self.idle()
                self.idle()
                rospy.signal_shutdown("Exit")

            if self.localise_before_decision:
                self.localise()
                self.track_map.location = self.track_map.locate(self.x,self.y,self.yaw)
                if self.track_map.location != self.full_path[self.decisionsI]:
                    self.plan_new_path()

            self.intersectionDecision = self.decisions[self.decisionsI] #replace this with service call
            self.decisionsI+=1
            # print(self.full_path[self.decisionsI],self.planned_path[0])
            if self.full_path[self.decisionsI] == self.planned_path[0]: #this is assuming that the destination of the maneuver is reached
                self.planned_path.pop(0)
            if self.intersectionDecision == 8:
                self.intersectionDecision = 0
                self.hw = True
            elif self.intersectionDecision == 9:
                self.intersectionDecision = 1
                self.hw = True
            elif self.intersectionDecision == 10:
                print("entering roundabout -> 8")
                self.intersectionDecision = -1 #reset
                self.state = 8
                return 1
            if self.intersectionDecision == 0: #left
                self.trajectory = self.left_trajectory
            elif self.intersectionDecision == 1: #straight
                self.trajectory = self.straight_trajectory
            elif self.intersectionDecision == 2: #right
                self.trajectory = self.right_trajectory
                if self.cp:
                    self.cp = False
            else:
                self.plan_new_path()
                print("self.intersectionDecision id wrong: ",self.intersectionDecision)
            print("intersection decision: going " + self.decisionList[self.intersectionDecision])
        if self.initialPoints is None:
            self.set_current_angle()
            # print("current orientation: ", self.directions[self.orientation], self.orientations[self.orientation])
            # print("destination orientation: ", self.destinationOrientation, self.destinationAngle)
            self.initialPoints = np.array([self.x, self.y])
            # print("initialPoints points: ", self.initialPoints)
            self.odomX, self.odomY = 0.0, 0.0 #reset x,y
            self.timerodom = rospy.Time.now()
            self.intersectionState = 0 #adjusting angle:0, trajectory following:1, adjusting angle2: 2..
            self.adjustYawError = 0.2 if self.intersectionDecision!=1 else 0.05
        self.odometry()
        poses = np.array([self.odomX,self.odomY])
        poses = poses.dot(self.rotation_matrices[self.orientation])
        x,y = poses[0], poses[1]
        # print("position: ",x,y)
        if self.intersectionState==0: #adjusting
            error = self.yaw-self.currentAngle
            if error>np.pi:
                error-=2*np.pi
            elif error<-np.pi:
                error+=2*np.pi
            # print("yaw, curAngle, error: ", self.yaw, self.currentAngle, error)
            if abs(error) <= self.adjustYawError:
                self.intersectionState+=1 #done adjusting
                # print("done adjusting angle. Transitioning to trajectory following")
                self.error_sum = 0 #reset pid errors
                self.last_error = 0
                return 0
            else:
                self.publish_cmd_vel(self.pid(error), self.maxspeed)
                return 0
        elif self.intersectionState==1: #trajectory following
            desiredY = self.trajectory(x)
            error = y - desiredY
            # print("x, y_error: ",x,abs(error))
            arrived = abs(self.yaw-self.destinationAngle) <= 0.15 or abs(self.yaw-self.destinationAngle) >= 6.13
            if self.intersectionDecision == 1:
                arrived = arrived and abs(x)>=1 and abs(y-self.offsets_y[self.intersectionDecision]) < 0.2
            # if self.roadblock:
            #     arrived = arrived and error < 0.2
            # print("yaw_error: ")
            # print(str(self.yaw-self.destinationAngle))
            if arrived:
                # print("trajectory done.")
                self.doneManeuvering = True
                self.last_error2 = 0 #reset pid errors
                self.error_sum2 = 0
                return 0
            # steering_angle = self.pid2(error)
            # print("steering: ",steering_angle)
            # print("x, y, desiredY, angle, steer: ", x, y, desiredY, self.yaw, steering_angle*180/3.14159)
            self.publish_cmd_vel(self.pid2(error), self.maxspeed)
            return 0
    
    def approachCrosswalk(self):
        #Transition events
        if self.timer is None: #start timer.
            # 13s*0.66*0.135m/s = 1.16m
            print("slowing down to "+str(0.8*self.maxspeed)+"m/s")
            t = 1.16/(0.8*self.maxspeed) # calculate time based on speed
            self.timer = rospy.Time.now() + rospy.Duration(t)
        if rospy.Time.now() >= self.timer:
            print("crosswalk passed, speed back up to "+str(self.maxspeed)+"m/s")
            self.timer = None #reset timer
            self.state = 0
            return 1
        elif self.pedestrian_appears():
            print("pedestrian appears!!! -> state 5")
            self.timer = None #reset timer
            self.history = self.state
            self.state = 5
            self.timerPedestrian = rospy.Time.now()+rospy.Duration(2.5)
            return 1
        #Action: slow down
        # Publish the steering command
        self.publish_cmd_vel(self.get_steering_angle(offset=40), self.maxspeed*0.8) #Slower
        return 0
    
    def stopPedestrian(self):
        if not self.pedestrian_appears():
            if rospy.Time.now() > self.timerPedestrian:
                self.timerPedestrian = None
                self.state = self.history if self.history is not None else 0
                self.history = None
                return 1
        else:
            # print("pedestrian appears!!!")
            self.timerPedestrian = rospy.Time.now()+rospy.Duration(2.5)
        #Action: idle
        self.idle()
        return 0 
    
    def highway(self):
        # if self.highway_exit_detected():
        #     if self.entering_roundabout(): #check this
        #         self.state = 1 #should be approaching roundabout state similar to approachInt
        #         return 1
        #     else: #go to approachInt
        #         self.intersectionStop = False
        #         self.state = 1
        #         return 1

        #go to left side of highway
        if self.timer1 is None:
            if self.highwayRight:
                self.timer1 = rospy.Time.now() + rospy.Duration(7.35)
                self.timer0 = None
                print("go for 12s")
        elif rospy.Time.now() >= self.timer1:
            if self.flag1:
                self.highwayYaw = self.yaw
                self.flag1 = False
                print("highwayYaw: ", self.highwayYaw)
            if self.highwayRight:
                error = self.yaw - self.highwayYaw
                if error>np.pi:
                    error-=2*np.pi
                elif error<-np.pi:
                    error+=2*np.pi
                if abs(error) > 0.175*np.pi:
                    if self.timer0 is None:
                        print("done turning. now go straight for 1.5s")
                        self.timer0 = rospy.Time.now() + rospy.Duration(0.47)
                    if rospy.Time.now() >= self.timer0:
                        self.timer0 = None
                        self.timer1 = None
                        self.highwayRight = False
                        self.flag1 = True
                    self.publish_cmd_vel(0, self.highwaySpeed*0.7)
                self.publish_cmd_vel(-23, self.highwaySpeed*0.7)
                return 0

        if self.decisionsI < len(self.decisions):
            if self.decisions[self.decisionsI] == 14 and abs(self.yaw-0.15) <= 0.05: #tune this
                self.doneManeuvering = False
                self.timer0 = None
                self.flag1 = True
                self.highwaySpeed = self.maxspeed*1.33
                self.state = 12
                self.highwaySide = 1
                return 1
        if self.ArrivedAtStopline:
            self.doneManeuvering = False #set to false before entering state 3
            self.timer0 = None
            self.flag1 = True
            self.highwaySpeed = self.maxspeed*1.33
            self.state = 3
            self.highwaySide = 1
            return 1
        if self.timer2 is not None and rospy.Time.now() >= self.timer2 and self.highwaySide == 1:
            #go back to right side
            print("timer2 expired & no car in sight. going back to right side")
            self.timerO = None
            self.timer2 = None
            self.history = self.state
            self.state = 7 #switch lane
            return 1
        # car_sizes =  self.get_car_size()
        # num = len(car_sizes)
        # # detect car, check sizes, wait for 2s, then check again. 
        # # if 2 cars, size same, then stay idle
        # # if 1 car, overtake
        # if num > 0:
        #     if self.timerO is None:
        #         print("car detected, waiting for 2s to ascertain car's speed")
        #         self.timerO = rospy.Time.now() + rospy.Duration(1)
        #         self.numCars, self.firstDetectionSizes = num, car_sizes
        #     if rospy.Time.now() >= self.timerO:
        #         if self.highwaySide == 1:
        #             self.timer2 = rospy.Time.now() + rospy.Duration(3) #refresh timer
        #         if num < 2:
        #             if self.highwaySide == 1: #on right side
        #                 print("only one car, overtake, side is ", self.highwaySide) 
        #                 self.timerO = None
        #                 self.history = self.state
        #                 self.state = 7 #overtake
        #                 return 1
        #             else: # on left side.
        #                 carSizeRatio = car_sizes[0][2]/car_sizes[0][3]
        #                 if carSizeRatio >= 1.6: #means car is on other lane
        #                     print("car detected but it's on the right. ignore.")
        #                     self.publish_cmd_vel(self.get_steering_angle(offset = 37.5), self.highwaySpeed)
        #                     return 0
        #         # if self.flag1:
        #         #     print("there are two cars, checking if they are static")
        #         #     self.flag1 = False
        #         #     static = abs(self.firstDetectionSizes[0] - car_sizes[0]) < 7 #tune this
        #         #     if static:
        #         #         self.highwaySpeed = 0
        #         #         print("car is static")
        #         #     else:
        #         #         self.highwaySpeed = 0.66*self.maxspeed #tune this, tail the car
        #         #         print("car is moving, we tail")
        #         self.idle()
        #         return 0
        #     else:
        #         self.idle()
        #         return 0
        if self.pedestrian_appears():
            print("pedestrian appears!!! -> state 5")
            self.history = self.state
            self.state = 5
            self.timerPedestrian = rospy.Time.now()+rospy.Duration(2.5)
            return 1
        self.publish_cmd_vel(self.get_steering_angle(), self.maxspeed*1.33)
        return 0
    
    def lane_overtake(self):
        #/entry: checkDotted
        #action: overtake or wait
        if self.pedestrian_appears():
            print("pedestrian appears!!! -> state 5")
            self.history = self.state
            self.state = 5
            self.timerPedestrian = rospy.Time.now()+rospy.Duration(2.5)
            return 1
        if self.doneManeuvering:
            print("done overtaking. Back to lane following...")
            self.doneManeuvering = False #reset
            self.state = self.history if self.history is not None else 0
            # cooldown = 5 if self.history == 6 else 2 #6 is highway
            self.history = None
            self.initialPoints = None #reset initial points
            self.timerP = None
            if not self.overtake:
                self.highwaySide *= -1 
                if self.highwaySide == -1:
                    self.timer2 = rospy.Time.now() + rospy.Duration(self.laneOvertakeCD) #tune this
                    print("cooldown is ", self.laneOvertakeCD)
            self.overtake = False
            self.pl = 320
            self.overtakeAngle = np.pi/5
            self.laneOvertakeAngle = np.pi*0.175
            return 1
        if self.initialPoints is None:
            self.overtaking_angle = self.yaw 
            self.initialPoints = np.array([self.x, self.y])
            # print("initialPoints points: ", self.initialPoints)
            self.offset = 0
            # print("begin going straight for "+str(self.offset)+"m")
            self.odomX, self.odomY = 0.0, 0.0 #reset x,y
            self.timerodom = rospy.Time.now()
            self.intersectionState = 0 #going straight:0, trajectory following:1, adjusting angle2: 2..
            self.localise()
            if self.x > 11 and self.y < 4:
                print("inside curved region")
                print(self.full_path[self.decisionsI])
                p = self.full_path[self.decisionsI]
                if p == "track2N" or p == "roundabout" or p == "highwayS" or p == "curvedpath":
                    print("on inner side")
                    self.overtakeDuration = 1
                    self.laneOvertakeAngle = 0.06*np.pi if self.highwaySide == 1 else 0.357*np.pi
                    self.laneOvertakeCD = 5
                    print("overtake angle is ", self.laneOvertakeAngle)
                else:
                    print("on outer side")
                    self.overtakeDuration = 0.5 if self.highwaySide == 1 else 0.5
                    self.laneOvertakeAngle = 0.24*np.pi if self.highwaySide == 1 else 0.1*np.pi
                    self.laneOvertakeCD = 4
                    print("overtake angle is ", self.laneOvertakeAngle)
            else: 
                self.overtakeDuration = 1
                self.laneOvertakeCD = 2
                self.laneOvertakeAngle = np.pi*0.175
        if self.intersectionState==0: #adjusting
            error = self.yaw - (self.overtaking_angle)
            if error>np.pi:
                error-=2*np.pi
            elif error<-np.pi:
                error+=2*np.pi
            # print("yaw, error: ", self.yaw, error)
            if abs(error) >= self.laneOvertakeAngle:
                self.intersectionState += 1
                print("done adjusting angle 0!!")
                return 0
            self.publish_cmd_vel(-23*self.highwaySide, self.maxspeed)
            return 0
        elif self.intersectionState==1: #adjusting
            if self.timer1 is None:
                self.timer1 = rospy.Time.now() + rospy.Duration(self.overtakeDuration)
            if rospy.Time.now() >= self.timer1:
                print("done changinng lane!!")
                self.doneManeuvering = True
                self.timer1 = None
                self.error_sum = 0 #reset pid errors
                self.last_error = 0
                return 0
            # error = self.yaw - self.overtaking_angle
            # if self.history == 6: #don't need to go back exactly at highway
            #     error -= np.pi/8
            # if error>np.pi:
            #     error-=2*np.pi
            # elif error<-np.pi:
            #     error+=2*np.pi
            # margin = 0.05 if self.history == 6 else 0.5
            # if abs(error) < margin:
            #     print("done changinng lane!!")
            #     self.doneManeuvering = True
            #     self.error_sum = 0 #reset pid errors
            #     self.last_error = 0
            #     return 0
            self.publish_cmd_vel(0, self.maxspeed)
        return 0
    
    def overtake(self):
        #/entry: checkDotted
        #action: overtake or wait
        if self.pedestrian_appears():
            print("pedestrian appears!!! -> state 5")
            self.history = self.state
            self.state = 5
            self.timerPedestrian = rospy.Time.now()+rospy.Duration(2.5)
            return 1
        if self.doneManeuvering:
            print("done overtaking. Back to lane following...")
            self.doneManeuvering = False #reset
            self.state = self.history if self.history is not None else 0
            cooldown = 5 if self.history == 6 else 2 #6 is highway
            self.history = None
            self.initialPoints = None #reset initial points
            self.timerP = None
            self.highwaySide *= -1
            self.pl = 320
            self.overtakeAngle = np.pi/5
            if self.highwaySide == -1:
                self.timer2 = rospy.Time.now() + rospy.Duration(cooldown) #tune this
            return 1
        if True: #change with left right overtaking if needed
            if self.initialPoints is None:
                self.overtaking_angle = self.yaw
                # print("current orientation: ", self.directions[self.orientation], self.orientations[self.orientation])
                # print("destination orientation: ", self.destinationOrientation, self.destinationAngle)
                self.initialPoints = np.array([self.x, self.y])
                # print("initialPoints points: ", self.initialPoints)
                self.offset = 0
                # print("begin going straight for "+str(self.offset)+"m")
                self.odomX, self.odomY = 0.0, 0.0 #reset x,y
                self.timerodom = rospy.Time.now()
                self.intersectionState = 0 #going straight:0, trajectory following:1, adjusting angle2: 2..
            if self.intersectionState==0: #adjusting
                error = self.yaw - (self.overtaking_angle + self.overtakeAngle*self.highwaySide)
                if error>np.pi:
                    error-=2*np.pi
                elif error<-np.pi:
                    error+=2*np.pi
                # print("yaw, error: ", self.yaw, error)
                if abs(error) <= 0.05:
                    self.intersectionState += 1
                    print("done adjusting angle 0!!")
                    return 0
                self.publish_cmd_vel(-23*self.highwaySide, self.maxspeed*0.8)
                return 0
            elif self.intersectionState==1: #adjusting
                error = self.yaw - self.overtaking_angle
                if self.history == 6: #don't need to go back exactly at highway
                    error -= np.pi/8
                if error>np.pi:
                    error-=2*np.pi
                elif error<-np.pi:
                    error+=2*np.pi
                margin = 0.05 if self.history == 6 else 0.5
                if abs(error) < margin:
                    print("done changinng lane!!")
                    self.doneManeuvering = True
                    self.error_sum = 0 #reset pid errors
                    self.last_error = 0
                    return 0
                self.publish_cmd_vel(23*self.highwaySide, self.maxspeed*0.8)
            return 0
            #         self.intersectionState += 1
            #         # print("done adjusting angle!!")
            #     self.publish_cmd_vel(23*self.highwaySide, self.maxspeed*0.8)
            #     return 0
            # elif self.intersectionState==2: #adjusting
            #     error = self.yaw - (self.overtaking_angle - self.overtakeAngle)
            #     if error>np.pi:
            #         error-=2*np.pi
            #     elif error<-np.pi:
            #         error+=2*np.pi
            #     # print("yaw, error: ", self.yaw, error)
            #     if abs(error) <= 0.05:
            #         self.intersectionState +=1
            #         # print("done adjusting angle!!")
            #     self.publish_cmd_vel(23, self.maxspeed*0.9)
            #     return 0
            # elif self.intersectionState==3: #adjusting
            #     error = self.yaw - self.overtaking_angle
            #     if error>np.pi:
            #         error-=2*np.pi
            #     elif error<-np.pi:
            #         error+=2*np.pi
            #     # print("yaw, error: ", self.yaw, error)
            #     if abs(error) <= 0.05:
            #         # print("done adjusting angle!!")
            #         self.doneManeuvering = True
            #         self.error_sum = 0 #reset pid errors
            #         self.last_error = 0
            #     self.publish_cmd_vel(-23, self.maxspeed*0.9)
            #     return 0
    
    def roundabout(self):
        if self.pedestrian_appears():
            print("pedestrian appears!!! -> state 5")
            self.history = self.state
            self.state = 5
            self.timerPedestrian = rospy.Time.now()+rospy.Duration(2.5)
            return 1
        if self.doneManeuvering:
            print("done roundabout maneuvering. Back to lane following...")
            self.doneManeuvering = False #reset
            if self.rdbDecision == 12:
                print("entering highway -> 6")
                self.state = 6
            else:
                self.state = 0 #go back to lane following
            self.rdbDecision = -1
            self.initialPoints = None #reset initial points
            self.timerP = None
            self.pl = 320
            return 1
        elif self.rdbDecision < 0:
            if self.decisionsI >= len(self.decisions):
                self.idle()
                self.idle()
                self.idle()
                rospy.signal_shutdown("Exit")

            if self.localise_before_decision:
                self.localise()
                self.track_map.location = self.track_map.locate(self.x,self.y,self.yaw)
                if self.track_map.location != self.full_path[self.decisionsI]:
                    self.plan_new_path()

            self.rdbDecision = self.decisions[self.decisionsI] #replace this with service call
            self.decisionsI+=1
            # print(self.full_path[self.decisionsI],self.planned_path[0])
            if self.full_path[self.decisionsI] == self.planned_path[0]: #this is assuming that the destination of the maneuver is reached
                self.planned_path.pop(0)
            #could place them in set_current_angle()
            if self.rdbDecision == 11: #E
                self.rdbExitYaw = np.pi/4 #change this depending on implementation
            elif self.rdbDecision == 12: #S
                self.rdbExitYaw = 7*np.pi/4 #change this depending on implementation
            elif self.rdbDecision == 13: #W
                self.rdbExitYaw = 5*np.pi/4 #change this depending on implementation
            else:
                self.plan_new_path()
                print("self.rdbDecision id wrong: ",self.rdbDecision)
            print("roundabout decision: going " + self.decisionList[self.rdbDecision])
            self.trajectory = self.rdb_trajectory
        if self.initialPoints is None:
            self.set_current_angle()
            # print("current orientation: ", self.directions[self.orientation], self.orientations[self.orientation])
            # print("destination orientation: ", self.destinationOrientation, self.destinationAngle)
            self.initialPoints = np.array([self.x, self.y])
            # print("initialPoints points: ", self.initialPoints)
            # self.rdbTransf = -self.orientations[self.orientation]
            self.odomX, self.odomY = 0.0, 0.0 #reset x,y
            self.timerodom = rospy.Time.now()
            self.offset = 0.4
            self.intersectionState = 0#adjusting angle:0, trajectory following:1, adjusting angle2: 2..
        self.odometry()
        poses = np.array([self.odomX,self.odomY])
        poses = poses.dot(self.rotation_matrices[self.orientation])
        x,y = poses[0], poses[1]
        if self.intersectionState==0: #adjusting
            error = self.yaw-self.currentAngle
            if error>np.pi:
                error-=2*np.pi
            elif error<-np.pi:
                error+=2*np.pi
            if x >= self.offset:
                self.intersectionState+=1
                self.odomX, self.odomY = 0.0, 0.0 #reset x,y
                self.timerodom = rospy.Time.now()
                self.error_sum = 0 #reset pid errors
                self.last_error = 0
                print("done going straight. Transitioning to trajectory following")
                # print("current angle, destination: ", self.yaw, self.destinationAngle)
            self.publish_cmd_vel(self.pid(error), self.maxspeed*0.9)
            return 0
        elif self.intersectionState==1:
            self.publish_cmd_vel(15, self.maxspeed*0.9)
            yaw = self.currentAngle-1
            yaw = yaw if yaw>0 else (6.2831853+yaw)
            arrived = abs(self.yaw-yaw) <= 0.1
            if arrived:
                print("trajectory done. adjusting angle")
                self.intersectionState += 1
                return 0
            return 0
        elif self.intersectionState==2: #trajectory following
            # desiredY = self.trajectory(x,self.rdbTransf)
            # error = y - desiredY
            # print("x,y,y_error: ",x,y,error)
            # self.publish_cmd_vel(self.pid2(error), self.maxspeed*0.9)
            self.publish_cmd_vel(-0.37, self.maxspeed*0.9)
            arrived = abs(self.yaw-self.rdbExitYaw) <= 0.1
            if arrived:
                print("trajectory done. adjusting angle")
                self.intersectionState += 1
                # self.last_error2 = 0 #reset pid errors
                # self.error_sum2 = 0
                return 0
            return 0
        elif self.intersectionState == 3: #adjust angle 2
            error = self.yaw-(self.rdbExitYaw-np.pi/4)
            if error>np.pi:
                error-=2*np.pi
            elif error<-np.pi:
                error+=2*np.pi
            # print("yaw, destAngle, error: ", self.yaw, self.destinationAngle, error)
            if abs(error) <= 0.1:
                print("done roundabout maneuvering!!")
                self.doneManeuvering = True
                self.error_sum = 0 #reset pid errors
                self.last_error = 0
                return 0
            else:
                self.publish_cmd_vel(self.pid(error), self.maxspeed*0.9)
                return 0
        return 0

    def park(self):
        if self.pedestrian_appears():
            print("pedestrian appears!!! -> state 5")
            self.history = self.state
            self.state = 5
            self.timerPedestrian = rospy.Time.now()+rospy.Duration(2.5)
            return 1
        if self.doneParking:
            print("done parking maneuvering. Stopping vehicle...")
            self.doneParking = False #reset
            self.state = 11 #parked
            self.parkingDecision = -1 #3 = normal, 4 = parallel
            self.initialPoints = None #reset initial points
            self.timerP = None
            self.pl = 320
            self.parkSecond = False
            return 1
        elif self.parkingDecision < 0:
            if self.decisionsI >= len(self.decisions):
                self.idle()
                self.idle()
                self.idle()
                rospy.signal_shutdown("Exit")

            if self.localise_before_decision:
                self.localise()
                self.track_map.location = self.track_map.locate(self.x,self.y,self.yaw)
                if self.track_map.location != self.full_path[self.decisionsI]:
                    self.plan_new_path()

            self.parkingDecision = self.decisions[self.decisionsI] #replace this with service call
            self.decisionsI+=1
            # print(self.full_path[self.decisionsI],self.planned_path[0])
            if self.full_path[self.decisionsI] == self.planned_path[0]: #this is assuming that the destination of the maneuver is reached
                self.planned_path.pop(0)
            if self.parkingDecision == 3: #front parking
                self.trajectory = self.right_trajectory
            elif self.parkingDecision == 4: #parallel parking
                pass
            else:
                self.plan_new_path()
                print("self.parkingDecision id wrong: ",self.parkingDecision)
            print("parking decision: going " + self.decisionList[self.parkingDecision])
        if self.parkingDecision == 4: #parallel
            if self.initialPoints is None:
                self.set_current_angle()
                # print("current orientation: ", self.directions[self.orientation], self.orientations[self.orientation])
                # print("destination orientation: ", self.destinationOrientation, self.destinationAngle)
                self.initialPoints = np.array([self.x, self.y])
                # print("initialPoints points: ", self.initialPoints)
                self.offset = 1.975 if self.simulation else 2.1 + self.parksize

                carSizes = self.get_car_size(minSize = 40)
                # print("car sizes: ", carSizes)
                if len(carSizes)==0:
                    print("no car in sight. park in first spot")
                else:
                    if self.timer0 is None:
                        self.timer0 = rospy.Time.now() + rospy.Duration(3)
                        print("car detected. waiting for 3 seconds")
                    if rospy.Time.now() < self.timer0:
                        self.initialPoints = None
                        return 0
                    if len(carSizes) > 1:
                        carsDetected = 0
                        carHeights = [0,0]
                        carX = [0,0]
                        for [x, y ,width,height] in carSizes:
                            if carsDetected > 1:
                                break
                            if width/height<1.6:
                                carHeights[carsDetected] = height
                                carX[carsDetected] = x
                                carsDetected += 1
                        if carX[0] < carX[1]:
                            laneCarHeight = carHeights[0]
                            parkedCarHeight = carHeights[1]
                        else:
                            laneCarHeight = carHeights[1]
                            parkedCarHeight = carHeights[0]
                        print(f"lane car height: {laneCarHeight}, parked car height: {parkedCarHeight}")
                        if max(carHeights) < 57:
                            print("first spot likely empty. proceed to park there")
                        if laneCarHeight >= 60:
                            print("lane car too close. parking operation aborted...")
                            print(f"lane car found. W/H ratio is {width/height}. parking operation aborted...")
                            self.doneParking = False #reset
                            self.state = 0
                            self.parkingDecision = -1 #3 = normal, 4 = parallel
                            self.initialPoints = None #reset initial points
                            self.timerP = None
                            self.pl = 320
                            self.decisionsI+=1
                            return 1
                        elif parkedCarHeight >= 65:
                            self.parkSecond = True
                            print(f"1 car found. Size is {height}. likely in first spot. proceeding to park in second spot")
                            self.offset += 0.72
                        else:
                            print(f"1 car found. Size is {height}. likely in second spot. proceeding to park in first spot")
                    elif len(carSizes)==1:
                        [_,_,width,height] = carSizes[0]
                        if height < 60:
                            print(f"1 car found. W/H ratio is {width/height}. likely in lane. proceeding to park in first spot")
                        else:
                            print(f"1 car found. Size is {height}. likely in first spot. proceeding to park in second spot")
                            self.offset += 0.72
                print("offset is ", self.offset)
                self.odomX, self.odomY = 0.0, 0.0 #reset x,y
                print("begin going straight for "+str(self.offset)+"m")
                self.timerodom = rospy.Time.now()
                self.intersectionState = 0 #going straight:0, trajectory following:1, adjusting angle2: 2..
                self.timer0 = None

            self.odometry()
            poses = np.array([self.odomX, self.odomY])
            poses = poses.dot(self.rotation_matrices[self.orientation])
            x, y = poses[0], poses[1]
            # print("position: ",x,y)
            if self.intersectionState==0: #going straight
                error = self.yaw-self.currentAngle
                if error>np.pi:
                    error-=2*np.pi
                elif error<-np.pi:
                    error+=2*np.pi
                if x >= self.offset:
                    print("done going straight.")
                    self.intersectionState = 1
                    self.error_sum = 0 #reset pid errors
                    self.last_error = 0
                    # print("done going straight. begin adjusting angle...")
                    # print("current angle, destination: ", self.yaw, self.destinationAngle)
                self.publish_cmd_vel(self.pid(error), self.maxspeed*0.9)
                if not self.parkSecond:
                    if self.update_belief(x):
                        self.parkSecond = True
                        self.offset += 0.72
                        print("second spot detected. adjusting offset to ", self.offset)
                return 0
            if self.intersectionState==1: #adjusting
                error = self.yaw - self.destinationAngle
                if error>np.pi:
                    error-=2*np.pi
                elif error<-np.pi:
                    error+=2*np.pi
                # print("yaw, error: ", self.yaw, error)
                if abs(error) >= self.parallelParkAngle*np.pi/180:
                    self.intersectionState = 2
                    print(f"{self.parallelParkAngle} degrees...")
                self.publish_cmd_vel(23, -self.maxspeed)
                return 0
            elif self.intersectionState==2: #adjusting
                error = self.yaw - self.destinationAngle
                if error>np.pi:
                    error-=2*np.pi
                elif error<-np.pi:
                    error+=2*np.pi
                if abs(error) < 0.05:
                    # print("done")
                    self.doneParking = True
                    return 0
                self.publish_cmd_vel(-23, -self.maxspeed)
                return 0
            
        elif self.parkingDecision == 3: #front parking
            if self.initialPoints is None:
                self.set_current_angle()
                # print("current orientation: ", self.directions[self.orientation], self.orientations[self.orientation])
                # print("destination orientation: ", self.destinationOrientation, self.destinationAngle)
                self.initialPoints = np.array([self.x, self.y])
                # print("initialPoints points: ", self.initialPoints)
                self.offset = 0.0 if self.simulation else 0.2 + self.parksize
                # self.offset += 0.463 if self.carsize>0 else 0
                carSizes = self.get_car_size(minSize = 40)
                print("car sizes: ", carSizes)
                if len(carSizes)==0:
                    print("no car in sight. park in first spot")
                elif len(carSizes)==1:
                    [_, _, width,height] = carSizes[0]
                    if width/height<1.6:
                        if height<60:
                            print(f"1 car found. W/H ratio is {width/height}. likely in lane. proceeding to park in first spot")
                        else:
                            print(f"lane car found. W/H ratio is {width/height}. parking operation aborted...")
                            self.doneParking = False #reset
                            self.state = 0
                            self.parkingDecision = -1 #3 = normal, 4 = parallel
                            self.initialPoints = None #reset initial points
                            self.timerP = None
                            self.pl = 320
                            self.decisionsI+=1
                            return 1
                    elif height>65:
                        print(f"1 car found. Size is {height}. likely in first spot. proceeding to park in second spot")
                        self.offset += 0.457
                        self.parkSecond = True
                    else: 
                        print(f"1 car found. Size is {height}. likely in second spot. proceeding to park in first spot")
                else:
                    parkedCarDetected = False
                    laneCarDetected = False
                    laneCarHeight = 0
                    parkedCarHeight = 0
                    for [_,_,width,height] in carSizes:
                        if parkedCarDetected and laneCarDetected:
                            break
                        if width/height>1.6:
                            laneCarHeight = height
                            parkedCarDetected = True
                        else:
                            print(f"2 cars found. W/H ratio is {width/height}.")
                            laneCarHeight = height
                            laneCarDetected = True
                    if laneCarHeight >= 60:
                        print("lane car too close. parking operation aborted...")
                        print(f"lane car found. W/H ratio is {width/height}. parking operation aborted...")
                        self.doneParking = False #reset
                        self.state = 0
                        self.parkingDecision = -1 #3 = normal, 4 = parallel
                        self.initialPoints = None #reset initial points
                        self.timerP = None
                        self.pl = 320
                        self.decisionsI+=1
                        return 1
                    elif parkedCarHeight >= 65:
                        print(f"1 car found. Size is {height}. likely in first spot. proceeding to park in second spot")
                        self.offset += 0.457
                    else:
                        print(f"1 car found. Size is {height}. likely in second spot. proceeding to park in first spot")
                self.carsize = 0
                print("begin going straight for "+str(self.offset)+"m")
                self.odomX, self.odomY = 0.0, 0.0 #reset x,y
                self.timerodom = rospy.Time.now()
                self.intersectionState = 1 #going straight:0, trajectory following:1, adjusting angle2: 2..
            self.odometry()
            poses = np.array([self.odomX, self.odomY])
            poses = poses.dot(self.rotation_matrices[self.orientation])
            x, y = poses[0], poses[1]
            # print("position: ",x,y)
            if self.intersectionState==0: #adjusting
                if abs(x)>=self.offset:
                    self.intersectionState+=1 #done adjusting
                    self.odomX, self.odomY = 0.0, 0.0 #reset x,y
                    self.timerodom = rospy.Time.now()
                    # print("done going straight. Transitioning to trajectory following")
                    # print(f"current odom position: ({self.odomX},{self.odomY})")
                    self.error_sum = 0 #reset pid errors
                    self.last_error = 0
                    return 0
                else:
                    self.publish_cmd_vel(self.get_steering_angle(), self.maxspeed)
                    # error = self.yaw-self.currentAngle
                    # self.publish_cmd_vel(self.pid(error), self.maxspeed)
                    # print(str(x))
                    if not self.parkSecond:
                        if self.update_belief(x):
                            self.parkSecond = True
                            self.offset += 0.457
                    return 0
            elif self.intersectionState==1: #trajectory following
                desiredY = self.trajectory(x)
                error = y - desiredY
                # print("x, y error: ",x,abs(error) )
                arrived = abs(self.yaw-self.destinationAngle) <= 0.3
                if arrived:# might need to change
                    # print("trajectory done. adjust angle round 2")
                    self.intersectionState += 1
                    self.last_error2 = 0 #reset pid errors
                    self.error_sum2 = 0
                    return 0
                # steering_angle = self.pid2(error)
                # print("x, y, desiredY, angle: ", x, y, desiredY, steering_angle*57.29578)
                self.publish_cmd_vel(self.pid2(error), self.maxspeed)
                return 0
            elif self.intersectionState == 2: #adjust angle 2
                error = self.yaw-self.destinationAngle
                if error>np.pi:
                    error-=2*np.pi
                elif error<-np.pi:
                    error+=2*np.pi
                # print("yaw, destAngle, error: ", self.yaw, self.destinationAngle, error)
                if abs(error) <= 0.05:
                    # print("done adjusting angle!!")
                    # print("adjusting position to y between 0.4-0.5")
                    self.intersectionState += 1
                    self.error_sum = 0 #reset pid errors
                    self.last_error = 0
                    return 0
                else:
                    if abs(y)<0.4: #adjust forward
                        self.publish_cmd_vel(self.pid(error), self.maxspeed*0.75)
                        self.parkAdjust = True
                    elif abs(y)>0.5: #adjust backward
                        self.publish_cmd_vel(-self.pid(error), -self.maxspeed*0.75)
                        self.parkAdjust = False
                    elif self.parkAdjust:
                        self.publish_cmd_vel(self.pid(error), self.maxspeed*0.75)
                    else:
                        self.publish_cmd_vel(-self.pid(error), -self.maxspeed*0.75)
                    return 0
            elif self.intersectionState == 3: #adjust position
                if abs(y)<0.4:
                    self.publish_cmd_vel(0, self.maxspeed*0.75)
                    return 0
                elif abs(y)>0.5:
                    self.publish_cmd_vel(0, -self.maxspeed*0.75)
                    return 0
                else:
                    # print("done adjusting position.")
                    # print(f"current odom position: ({self.odomX},{self.odomY})")
                    self.doneParking = True
                    return 0
    def update_belief(self, x):
        carSizes = self.get_car_size(minSize = 40)
        if len(carSizes)==0:
            return False
        if self.parkingDecision == 4: #parallel
            if len(carSizes) > 1:
                carsDetected = 0
                carHeights = [0,0]
                carX = [0,0]
                for [x, y ,width,height] in carSizes:
                    if carsDetected > 1:
                        break
                    if width/height<1.6:
                        carHeights[carsDetected] = height
                        carX[carsDetected] = x
                        carsDetected += 1
                if carX[0] < carX[1]:
                    # laneCarHeight = carHeights[0]
                    parkedCarHeight = carHeights[1]
                else:
                    # laneCarHeight = carHeights[1]
                    parkedCarHeight = carHeights[0]
                distanceToParked = self.carsize_to_distance(parkedCarHeight)
                if parkedCarHeight!= 0 and distanceToParked+x <= 1.5:
                    print(f"2 cars found. distance is {distanceToParked+x}. likely in first spot.")
                    return True
            elif len(carSizes)==1:
                [_,_,width,height] = carSizes[0]
                distanceToParked = self.carsize_to_distance(height)
                if height!= 0 and distanceToParked+x <= 1.5:
                    print(f"1 car found. distance is {distanceToParked+x}. likely in first spot.")
                    return True
            return False
        elif self.parkingDecision == 3: #forward
            carSizes = self.get_car_size(minSize = 40, conf_thresh = 0.375)
            if len(carSizes)==0:
                return False
            elif len(carSizes)==1:
                [_, _, width,height] = carSizes[0]
                if width/height>1.6:
                    distance = self.carsize_to_distance(height)
                    if distance+x <= 1.35:
                        print(f"1 car found. Size is {height}. likely in first spot. proceeding to park in second spot")
                        return True
                return False
            else:
                parkedCarDetected = False
                laneCarDetected = False
                laneCarHeight = 0
                parkedCarHeight = 0
                for [_,_,width,height] in carSizes:
                    if parkedCarDetected and laneCarDetected:
                        break
                    if width/height>1.6:
                        laneCarHeight = height
                        parkedCarDetected = True
                    else:
                        print(f"2 cars found. W/H ratio is {width/height}.")
                        laneCarHeight = height
                        laneCarDetected = True
                distanceToParked = self.carsize_to_distance(parkedCarHeight)
                if parkedCarHeight!= 0 and distanceToParked+x <= 1.35:
                    print(f"2 cars found. Size is {laneCarHeight}. likely in first spot. proceeding to park in second spot")
                    return True
        return False
    def exitPark(self):
        if self.pedestrian_appears():
            print("pedestrian appears!!! -> state 5")
            self.history = self.state
            self.state = 5
            self.timerPedestrian = rospy.Time.now()+rospy.Duration(2.5)
            return 1
        if self.doneManeuvering:
            print("done exit maneuvering. Back to lane following...")
            self.doneManeuvering = False #reset
            self.exitDecision = -1 #reset
            self.state = 0 #go back to lane following
            self.initialPoints = None #reset initial points
            self.timerP = None
            self.exitCD = rospy.Time.now() + rospy.Duration(3.57)
            self.pl = 320
            self.adjustYawError = 0.2
            return 1
        elif self.exitDecision < 0:
            if self.decisionsI >= len(self.decisions):
                self.idle()
                self.idle()
                self.idle()
                rospy.signal_shutdown("Exit")

            if self.localise_before_decision:
                self.localise()
                self.track_map.location = self.track_map.locate(self.x,self.y,self.yaw)
                if self.track_map.location != self.full_path[self.decisionsI]:
                    self.plan_new_path()

            self.exitDecision = self.decisions[self.decisionsI] #replace this with service call
            self.decisionsI+=1
            # print(self.full_path[self.decisionsI],self.planned_path[0])
            if self.full_path[self.decisionsI] == self.planned_path[0]: #this is assuming that the destination of the maneuver is reached
                self.planned_path.pop(0)
            print("exit decision: going ") #+ self.exitDecisions[self.exitDecision])
            if self.exitDecision == 5: #left exit
                self.trajectory = self.left_exit_trajectory
            elif self.exitDecision == 6: #right exit
                self.trajectory = self.right_exit_trajectory
            elif self.exitDecision == 7: #parallel exit
                pass
            else:
                self.plan_new_path()
                print("self.exitDecision id wrong: ",self.exitDecision)
            print("exit decision: going " + self.decisionList[self.exitDecision])
        if self.exitDecision != 7:
            if self.initialPoints is None:
                self.yaw=(self.yaw+3.14159)%(6.28318) #flip Yaw
                self.set_current_angle()
                # print("current orientation: ", self.directions[self.orientation], self.orientations[self.orientation])
                # print("destination orientation: ", self.destinationOrientation, self.destinationAngle)
                self.initialPoints = np.array([self.x, self.y])
                # print("initialPoints points: ", self.initialPoints)
                self.odomX, self.odomY = 0.0, 0.0 #reset x,y
                self.timerodom = rospy.Time.now()
                self.adjustYawError = 0.40 if self.exitDecision==5 else 0.07
                self.intersectionState = 1
            self.yaw=(self.yaw+3.14159)%(6.28318) #flip Yaw
            self.odometry()
            poses = np.array([self.odomX,self.odomY])
            poses = poses.dot(self.rotation_matrices[self.orientation])
            x,y = -poses[0], -poses[1]
            # print("position: ",x,y)
            if self.intersectionState==1: #trajectory following
                desiredY = self.trajectory(x)
                error = y - desiredY
                # print("x, y_error: ",x,abs(error) )
                arrived = abs(self.yaw-self.destinationAngle) <= self.adjustYawError
                # print("yaw_error: ")
                # print(str(self.yaw-self.destinationAngle))
                if arrived:
                    # print("trajectory done. adjust angle round 2")
                    self.intersectionState += 1
                    self.last_error2 = 0 #reset pid errors
                    self.error_sum2 = 0
                    return 0
                # steering_angle = self.pid2(error)
                # print("steering: ",steering_angle)
                # print("x, y, desiredY, angle, steer: ", x, y, desiredY, self.yaw, steering_angle*180/3.14159)
                self.publish_cmd_vel(-self.pid2(error), -self.maxspeed)
                return 0
            elif self.intersectionState == 2: #adjust angle 2
                error = self.yaw-self.destinationAngle
                if error>np.pi:
                    error-=2*np.pi
                elif error<-np.pi:
                    error+=2*np.pi
                # print("yaw, destAngle, error: ", self.yaw, self.destinationAngle, error)
                if abs(error) <= self.adjustYawError:
                    # print("done adjusting angle!!")
                    self.doneManeuvering = True
                    self.error_sum = 0 #reset pid errors
                    self.last_error = 0
                    return 0
                else:
                    self.publish_cmd_vel(self.pid(error), self.maxspeed)
                    return 0
        elif self.exitDecision == 7:
            if self.initialPoints is None:
                self.set_current_angle()
                # print("current orientation: ", self.directions[self.orientation], self.orientations[self.orientation])
                # print("destination orientation: ", self.destinationOrientation, self.destinationAngle)
                self.initialPoints = np.array([self.x, self.y])
                # print("initialPoints points: ", self.initialPoints)
                self.offset = 0
                # print("begin going straight for "+str(self.offset)+"m")
                self.odomX, self.odomY = 0.0, 0.0 #reset x,y
                self.timerodom = rospy.Time.now()
                self.intersectionState = 0 #going straight:0, trajectory following:1, adjusting angle2: 2..
            self.odometry()
            poses = np.array([self.odomX, self.odomY])
            poses = poses.dot(self.rotation_matrices[self.orientation])
            x, y = poses[0], poses[1]
            # print("position: ",x,y)
            if self.intersectionState==0: #going straight
                error = self.yaw-self.currentAngle
                if x >= self.offset:
                    # print("done going straight. begin adjusting angle...")
                    self.intersectionState = 1
                    self.error_sum = 0 #reset pid errors
                    self.last_error = 0
                    # print("current angle, destination: ", self.yaw, self.destinationAngle)
                self.publish_cmd_vel(self.pid(error), self.maxspeed)
                return 0
            if self.intersectionState==1: #adjusting
                error = self.yaw - self.destinationAngle
                if error>np.pi:
                    error-=2*np.pi
                elif error<-np.pi:
                    error+=2*np.pi
                # print("yaw, error: ", self.yaw, error)
                if abs(error) >= self.parallelParkAngle*np.pi/180:
                    self.intersectionState = 2
                    # print(f"{self.parallelParkAngle} degrees...")
                self.publish_cmd_vel(-23, self.maxspeed)
                return 0
            elif self.intersectionState==2: #adjusting
                error = self.yaw - self.destinationAngle
                if error>np.pi:
                    error-=2*np.pi
                elif error<-np.pi:
                    error+=2*np.pi
                if abs(error) < 0.05:
                    # print("done adjusting angle!!")
                    self.doneManeuvering = True
                    self.error_sum = 0 #reset pid errors
                    self.last_error = 0
                    return 0
                self.publish_cmd_vel(23, self.maxspeed)
                return 0
    
    def curvedpath(self):
        if self.pedestrian_appears():
            print("pedestrian appears!!! -> state 5")
            self.history = self.state
            self.state = 5
            self.timerPedestrian = rospy.Time.now()+rospy.Duration(2.5)
            return 1
        if self.doneManeuvering:
            print("done curvedpath maneuvering.")
            self.doneManeuvering = False #reset
            self.intersectionDecision = -1 #reset
            self.initialPoints = None #reset initial points
            self.pl = 320
            self.state = 0
            self.cp = True
            return 1
        elif self.intersectionDecision < 0:
            if self.decisionsI >= len(self.decisions):
                self.idle()
                self.idle()
                self.idle()
                rospy.signal_shutdown("Exit")

            if self.localise_before_decision:
                self.localise()
                self.track_map.location = self.track_map.locate(self.x,self.y,self.yaw)
                if self.track_map.location != self.full_path[self.decisionsI]:
                    self.plan_new_path()

            self.intersectionDecision = self.decisions[self.decisionsI] #replace this with service call
            self.decisionsI+=1
            # print(self.full_path[self.decisionsI],self.planned_path[0])
            if self.full_path[self.decisionsI] == self.planned_path[0]: #this is assuming that the destination of the maneuver is reached
                self.planned_path.pop(0)
            if self.intersectionDecision == 14:
                pass
            else:
                self.plan_new_path()
                print("self.intersectionDecision id wrong: ",self.intersectionDecision)
            print("highway decision: going " + self.decisionList[self.intersectionDecision])
        if self.initialPoints is None:
            self.set_current_angle()
            # print("current orientation: ", self.directions[self.orientation], self.orientations[self.orientation])
            # print("destination orientation: ", self.destinationOrientation, self.destinationAngle)
            self.initialPoints = np.array([self.x, self.y])
            # print("initialPoints points: ", self.initialPoints)
            self.offset = 1.5 #tune this
            self.odomX, self.odomY = 0.0, 0.0 #reset x,y
            self.timerodom = rospy.Time.now()
            self.intersectionState = 0
        self.odometry()
        poses = np.array([self.odomX,self.odomY])
        poses = poses.dot(self.rotation_matrices[self.orientation])
        x,y = poses[0], poses[1]
        # print("position: ",x,y)
        if self.intersectionState==0: #adjusting
            error = self.yaw-self.currentAngle
            if error>np.pi:
                error-=2*np.pi
            elif error<-np.pi:
                error+=2*np.pi
            # print("yaw, curAngle, error: ", self.yaw, self.currentAngle, error)
            if x >= self.offset:
                print("trajectory done.")
                self.doneManeuvering = True
                self.error_sum = 0 #reset pid errors
                self.last_error = 0
                return 0
            else:
                self.publish_cmd_vel(self.pid(error), self.maxspeed)
                return 0
    
    #transition events
    def entering_roundabout(self):
        return self.object_detected(3)
    def stop_sign_detected(self):
        return self.object_detected(2)
    def highway_entrance_detected(self):
        return self.object_detected(1)
    def highway_exit_detected(self):
        return self.object_detected(7)
    def light_detected(self):
        return self.object_detected(9)
    def parking_detected(self):
        return self.object_detected(4)
    def is_green(self):
        self.orientation = np.argmin([abs(self.yaw),abs(self.yaw-1.5708),abs((self.yaw)-3.14159),abs(self.yaw-4.71239),abs(self.yaw-6.28319)])%4
        if self.simulation:
            if self.orientation==1 or self.orientation==3: #N or S
                topic = 'start' #'anitmaster'
            else:
                topic = 'master' #'slave'
            try:
                self.idle()
                state=rospy.wait_for_message('/automobile/trafficlight/'+topic,Byte,timeout=1)#0=red,1=yellow,2=green
            except:
                print("traffic light timed out")
                return True
            return True if state.data == 2 else False
        else:
            if self.orientation==1 or self.orientation==3: #N or S
                # topic = 'start' #'anitmaster'
                try:
                    data, addr = self.sock.recvfrom(4096) # buffer size is 1024 bytes
                    dat = data.decode('utf-8')
                    dat = json.loads(dat)
                    ID = int(dat['id'])
                    state = int(dat['state'])
                    if (ID == 1) or (ID == 2):
                        return True if state == 2 else False
                    else:
                        return True if state == 0 else False
                except Exception as e:
                    if str(e) !="timed out":
                        print("Receiving data failed with error: " + str(e))
                        return False
            else:
                # topic = 'master' #'slave'
                try:
                    data, addr = self.sock.recvfrom(4096) # buffer size is 1024 bytes
                    dat = data.decode('utf-8')
                    dat = json.loads(dat)
                    ID = int(dat['id'])
                    state = int(dat['state'])
                    if (ID == 3) or (ID == 4):
                        return True if state == 2 else False
                    else:
                        return True if state == 0 else False
                except Exception as e:
                    if str(e) !="timed out":
                        print("Receiving data failed with error: " + str(e))
                        return False
    def crosswalk_sign_detected(self):
        return self.object_detected(5)
    def pedestrian_appears(self):
        return self.object_detected(11)
    def car_detected(self):
        return self.object_detected(12)
    def roadblock_detected(self):
        return self.object_detected(10)

    #controller functions
    def idle(self):
        # self.cmd_vel_pub(0.0, 0.0)
        # self.msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
        # self.cmd_vel_pub.publish(self.msg)
        if self.simulation:
            self.msg.data = '{"action":"1","speed":'+str(0.0)+'}'
            self.msg2.data = '{"action":"2","steerAngle":'+str(0.0)+'}'
            self.cmd_vel_pub.publish(self.msg)
            self.cmd_vel_pub.publish(self.msg2)
        else:
            self.msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
            self._write(self.msg)

    #odom helper functions
    def pid(self, error):
        # self.error_sum += error * self.dt
        dt = (rospy.Time.now()-self.timerpid).to_sec()
        # rospy.loginfo("time: %.4f", self.dt)
        self.timerpid = rospy.Time.now()
        derivative = (error - self.last_error) / dt
        output = self.kp * error + self.kd * derivative #+ self.ki * self.error_sum
        self.last_error = error
        return output
    def pid2(self, error):
        # self.error_sum2 += error * self.dt
        dt = (rospy.Time.now()-self.timerpid).to_sec()
        # rospy.loginfo("time: %.4f", self.dt)
        self.timerpid = rospy.Time.now()
        derivative = (error - self.last_error2) / dt
        output = self.kp2 * error + self.kd2 * derivative #+ self.ki2 * self.error_sum2
        self.last_error2 = error
        return output
    def odometry(self):
        dt = (rospy.Time.now()-self.timerodom).to_sec()
        self.timerodom = rospy.Time.now()
        magnitude = self.velocity*dt*self.odomRatio
        self.odomX += magnitude * math.cos(self.yaw)
        self.odomY += magnitude * math.sin(self.yaw)
        # print(f"odometry: speed={self.velocity}, dt={dt}, mag={magnitude}, cos={math.cos(self.yaw)}, X={self.odomX}, Y={self.odomY}")
    def set_current_angle(self):
        #0:left, 1:straight, 2:right, 3:parkF, 4:parkP, 5:exitparkL, 6:exitparkR, 7:exitparkP
        #8:enterhwLeft, 9:enterhwStright, 10:rdb, 11:exitrdbE, 12:exitrdbS, 13:exitrdbW, 14:curvedpath
        self.orientation = np.argmin([abs(self.yaw),abs(self.yaw-1.5708),abs((self.yaw)-3.14159),abs(self.yaw-4.71239),abs(self.yaw-6.28319)])%4
        self.currentAngle = self.orientations[self.orientation]
        if self.intersectionDecision == 0 or self.exitDecision == 5: #left
            self.destinationOrientation = self.directions[(self.orientation+1)%4]
            self.destinationAngle = self.orientations[(self.orientation+1)%4]
            return
        elif self.intersectionDecision == 1: #straight
            self.destinationOrientation = self.orientation
            self.destinationAngle = self.currentAngle
            return
        elif self.intersectionDecision == 2 or self.parkingDecision == 3 or self.exitDecision == 6: #right
            self.destinationOrientation = self.directions[(self.orientation-1)%4]
            self.destinationAngle = self.orientations[(self.orientation-1)%4]
            return
        elif self.parkingDecision == 4:
            self.destinationOrientation = self.directions[(self.orientation)%4]
            self.destinationAngle = self.orientations[(self.orientation)%4]
            return

    #trajectories
    def straight_trajectory(self, x):
        return 0
    def left_trajectory_real(self, x):
        return math.exp(3.57*x-4.3)
    def right_trajectory_real(self, x):
        return -math.exp(4*x-2.85)
    def left_exit_trajectory_real(self, x):
        return math.exp(4*x+2)
    def right_exit_trajectory_real(self, x):
        return -math.exp(4*x-3.05)
    def leftpark_trajectory(self, x):
        return math.exp(3.57*x-4.2) #real dimensions
    def left_trajectory_sim(self, x):
        return math.exp(3.57*x-4.53)
    def right_trajectory_sim(self, x):
        return -math.exp(3.75*x-3.33)
    def left_exit_trajectory_sim(self, x):
        return math.exp(3*x-0.75)
    def right_exit_trajectory_sim(self, x):
        return -math.exp(3.75*x-2.53)
    def rdb_trajectory(self, x, t):
        u = 0.5-math.pow(x-0.71,2)
        u = np.clip(u,0,255)
        yaw = self.yaw+t if self.yaw+t>0 else (6.2831853+self.yaw+t)
        return -math.sqrt(u)+0.25 if yaw<np.pi/2 or yaw>3*np.pi/2 else math.sqrt(u)+0.25

    #others
    def carsize_to_distance(self, height):
        # return 0.92523112 - 0.42423197*height + 0.28328111*height*height - 0.08428235*height*height*height
        return 84.89/(height - 3.137)
    def get_obj_size(self, obj_id = 10, minSize = None):
        sizes = []
        if minSize is None:
            minSize = self.min_sizes[obj_id]
        maxSize = self.max_sizes[obj_id]
        boxes = [self.box1, self.box2, self.box3, self.box4]
        for i in range(self.numObj):    
            if self.detected_objects[i]==obj_id: 
                box = boxes[i]
                conf = self.confidence[i]
                size = min(box[2], box[3]) if (obj_id == 12 or obj_id == 10) else max(box[2], box[3]) #height
                if size >= minSize and size <= maxSize and conf >= 0.753:
                    sizes.append(box)
            if i>=3:
                break
        return sizes
    def get_car_size(self, minSize = None, conf_thresh = None):
        obj_id = 12 #car id
        car_sizes = []
        thresh = 0.753 if conf_thresh is None else conf_thresh
        if minSize is None:
            minSize = self.min_sizes[obj_id]
        maxSize = self.max_sizes[obj_id]
        boxes = [self.box1, self.box2, self.box3, self.box4]
        for i in range(self.numObj):    
            if self.detected_objects[i]==obj_id: 
                box = boxes[i]
                conf = self.confidence[i]
                size = min(box[2], box[3]) #height
                if size >= minSize and size <= maxSize and conf >= thresh:
                    car_sizes.append(box)
            if i>=3: 
                break
        return car_sizes
    def object_detected(self, obj_id):
        if self.numObj >= 2:
            if self.detected_objects[0]==obj_id: 
                if self.check_size(obj_id,0):
                    return True
            elif self.detected_objects[1]==obj_id:
                if self.check_size(obj_id,1):
                    return True
        elif self.numObj == 1:
            if self.detected_objects[0]==obj_id: 
                if self.check_size(obj_id,0):
                    return True
        return False
    def check_size(self, obj_id, index):
        #checks whether a detected object is within a certain min and max sizes defined by the obj type
        box = self.box1 if index==0 else self.box2
        conf = self.confidence[index]
        size = max(box[2], box[3])
        if obj_id==12:
            size = min(box[2], box[3])
        if obj_id==10:
            conf_thresh = 0.35
        else:
            conf_thresh = 0.8
        return size >= self.min_sizes[obj_id] and size <= self.max_sizes[obj_id] and conf >= conf_thresh #check this
    def get_steering_angle(self,offset=30):
        """
        Determine the steering angle based on the lane center
        :param center: lane center
        :return: Steering angle in radians
        """
        # Calculate the steering angle in radians
        self.dt = (rospy.Time.now()-self.timerpid).to_sec()
        self.timerpid = rospy.Time.now()
        error = (self.center + offset - self.image_center)
        try:
            d_error = (error-self.last)/self.dt
        except:
            return 0
        self.last = error
        steering_angle = (error*self.p+d_error*self.d)
        return steering_angle
    def publish_cmd_vel_real(self, steering_angle, velocity = None, clip = True):
        """
        Publish the steering command to the cmd_vel topic
        :param steering_angle: Steering angle in radians
        """
        if velocity is None:
            velocity = self.maxspeed
        if clip:
            steering_angle = np.clip(steering_angle*180/np.pi, -22.9, 22.9)
        if self.toggle == 0:
            self.toggle = 1
            self.msg.data = '{"action":"1","speed":'+str(float("{:.4f}".format(velocity)))+'}'
        else:
            self.toggle = 0
            self.msg.data = '{"action":"2","steerAngle":'+str(float("{:.2f}".format(steering_angle)))+'}'
        self._write(self.msg)
    def publish_cmd_vel_sim(self, steering_angle, velocity = None, clip = True):
        if velocity is None:
            velocity = self.maxspeed
        if clip:
            steering_angle = np.clip(steering_angle*180/np.pi, -22.9, 22.9)
        self.msg.data = '{"action":"1","speed":'+str(velocity)+'}'
        self.msg2.data = '{"action":"2","steerAngle":'+str(float(steering_angle))+'}'
        # print(self.msg.data)
        # print(self.msg2.data)
        self.cmd_vel_pub.publish(self.msg)
        self.cmd_vel_pub.publish(self.msg2)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='State Machine for Robot Control.')
    parser.add_argument("--simulation", type=str, default=True, help="Run the robot in simulation or real life")
    parser.add_argument("--path", type=str, default="/paths/path.json", help="Planned path")
    parser.add_argument("--custom", type=str, default=False, help="Custom path")
    # args, unknown = parser.parse_known_args()
    args = parser.parse_args(rospy.myargv()[1:])
    s = args.simulation=="True"
    c = args.custom=="True"
    node = StateMachine(simulation=s,planned_path=args.path,custom_path=c)
    rospy.spin()