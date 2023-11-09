#!/usr/bin/env python3
import time
import numpy as np
import rospy
import threading
import argparse
from utility import Utility
from mpc_acados import Optimizer

class StateMachine:
    def __init__(self, mpc, utils):
        self.STATES = {"moving", "waiting for stop sign", "waiting for traffic light", "parking", "exiting parking"}
        self.state = 0
        self.mpc = mpc
        self.utils = utils
        debug = True
        gaz_bool = "_gazebo_"
        self.mpc.lock = threading.Lock()
        

    def spin_thread(self):
        rospy.spin()
    def run(self):
        pass

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--useEkf', action='store_true', help='Boolean for whether to use ekf node or not')
    parser.add_argument('--odom', action='store_true', help='Boolean for whether to use odometry or gps data')
    parser.add_argument('--sign', action='store_true', help='Boolean for whether to use sign or not')
    args = parser.parse_args()
    debug = True
    mpc = Optimizer(gazebo=True)
    gaz_bool = "_gazebo_"
    rospy.init_node("mpc_node")
    mpc.real_x = None
    mpc.real_y = None
    mpc.yaw = None
    mpc.lock = threading.Lock()
    rospy.init_node("mpc_node")
    utils = Utility(subLane=False, subImu=True, subModel=True, pubOdom=True, useEkf=args.useEkf, subSign=args.sign)
    print("gazebo!")
    def spin_thread():
        rospy.spin()
    thread = threading.Thread(target=spin_thread)
    thread.start()
    rate = rospy.Rate(1/mpc.T)
    # wait until the first state is received
    print("waiting for first state...")
    while not utils.initializationFlag:
        time.sleep(0.1)
    mpc.real_x = utils.gps_x
    mpc.real_y = utils.gps_y
    mpc.yaw = utils.yaw
    while (mpc.real_x is None or mpc.real_y is None or mpc.yaw is None):
        time.sleep(0.1)
    print("starting... x: ", mpc.real_x, ", y: ", mpc.real_y, ", yaw: ", mpc.yaw)
    mpc.detected_index = 0
    # stop when last waypoint is reached
    length = len(mpc.waypoints_x)
    mpc.target_waypoint_index = 0
    print("length: ", length)
    timer = rospy.Time.now()
    while True:
        if mpc.target_waypoint_index >= mpc.num_waypoints-1:
            break
        t = time.time()
        if args.sign:
            stop_sign_dist = utils.sign_detected(2)
            traffic_light_dist = utils.sign_detected(9)
            if rospy.Time.now() >= timer: 
                if stop_sign_dist>0 and stop_sign_dist<0.3:
                    print("stop sign detected at a distance of: ", stop_sign_dist)
                    dist = stop_sign_dist
                elif traffic_light_dist>0 and traffic_light_dist<0.3:
                    print("traffic light detected at a distance of: ", traffic_light_dist)
                    dist = traffic_light_dist
                
            distance = utils.object_detected(12)
            if distance>0 and mpc.target_waypoint_index>=mpc.detected_index:
                distance = max(distance, 0.75)-0.75 #0.75 is minimum distance
                print("car detected at a distance of: ", distance+0.75)
                offset = int(distance*mpc.density)
                mpc.detected_index = mpc.target_waypoint_index+20
                mpc.path.change_lane(mpc.target_waypoint_index+offset, mpc.detected_index+offset, mpc.wp_normals)
        with mpc.lock:
            if args.odom:
                mpc.update_current_state(x=utils.odomX, y=utils.odomY, yaw=utils.yaw)
                mpc.update_real_state(x=utils.gps_x, y=utils.gps_y, yaw=utils.yaw)
            else:
                mpc.update_current_state(x=utils.gps_x, y=utils.gps_y, yaw=utils.yaw)
        if debug:
            mpc.x_errors.append(utils.gps_x - mpc.next_trajectories[0, 0])
            mpc.y_errors.append(utils.gps_y - mpc.next_trajectories[0, 1])
            mpc.x_refs.append(mpc.next_trajectories[0, :])
            mpc.yaw_errors.append(mpc.current_state[2] - mpc.next_trajectories[0, 2])
        t_ = time.time()
        u_res = mpc.update_and_solve()
        t2 = time.time()- t_
        if debug:
            mpc.index_t.append(t2)
            mpc.integrate_next_states(u_res)
            mpc.t_c.append(mpc.t0)
            mpc.u_c.append(u_res)
            mpc.xx.append(mpc.real_state) if args.odom else mpc.xx.append(mpc.current_state)
            mpc.mpciter = mpc.mpciter + 1
        utils.steer_command = -float(u_res[1]*180/np.pi)
        utils.velocity_command = float(u_res[0])
        utils.publish_cmd_vel(steering_angle=utils.steer_command, velocity=utils.velocity_command)
        rate.sleep()
        if debug:
            print("i) ", mpc.mpciter,"ref_traj:", np.around(mpc.next_trajectories, decimals=2)[0], 
                "cur:", np.around(mpc.current_state, decimals=2), "ctrl:", 
                np.around(u_res, decimals=2), "idx:", mpc.target_waypoint_index, "time:", round(t2, 4),
                "kappa:", round(mpc.kappa[mpc.target_waypoint_index],2))
    if debug:
        stats = mpc.compute_stats()
        mpc.current_state = [utils.gps_x, utils.gps_y, utils.yaw]
        print("current state: ", mpc.current_state)
    # park_offset = 0.1
    # mpc.park(park_offset, utils=utils)
    # mpc.exit_park(utils=utils)
    print("done")
    try:
        utils.call_trigger_service()
    except:
        print("service not available")
    if debug:
        mpc.draw_result(stats)