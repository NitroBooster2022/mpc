#!/usr/bin/env python3

import time
from draw import Draw_MPC_tracking
import casadi as ca
import numpy as np
import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from utils.msg import IMU
import threading
import os

class MPC:
    def __init__(self):
        self.look_ahead = 0.6
        self.gazebo = True
        gaz_bool = ""
        if self.gazebo:
            gaz_bool = "_gazebo_"
            rospy.init_node("talker_p")
            self.pub = rospy.Publisher("automobile/command",String,queue_size=3)
            self.msg = String()  
            self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback, queue_size=3)
            self.imu_sub = rospy.Subscriber("/automobile/IMU", IMU, self.imu_callback, queue_size=3)
            self.real_x = None
            self.real_y = None
            self.yaw = None
            self.car_idx = None
            self.lock = threading.Lock()
        name = 'speedrun1'
        filepath = os.path.dirname(os.path.abspath(__file__))
        self.waypoints = np.load(os.path.join(filepath,name+'.npy'))
        self.waypoints_x = self.waypoints[0, 27:50]
        self.waypoints_y = self.waypoints[1, 27:50]
        self.waypoints_x = self.waypoints[0]
        self.waypoints_y = self.waypoints[1]
        self.x_max = np.max(self.waypoints_x)*1.1
        self.x_min = np.min(self.waypoints_x)*0.9
        self.y_max = np.max(self.waypoints_y)*1.1
        self.y_min = np.min(self.waypoints_y)*0.9
        print("x_max: ", self.x_max, ", x_min: ", self.x_min, ", y_max: ", self.y_max, ", y_min: ", self.y_min)
        # Constants
        self.latency = 0#.724
        self.L = 0.27
        self.T = 0.2415
        self.N = 2
        self.rob_diam = 0.3
        self.v_max = 0.5
        self.v_min = 0
        # self.x_max = 15
        # self.x_min = 0
        # self.y_min = 0
        # self.y_max = 15
        self.steer_max = 0.4003
        self.v_ref = 0.35
        #compute angle between first and second waypoint
        theta_ref = np.arctan2(self.waypoints_y[1] - self.waypoints_y[0], self.waypoints_x[1] - self.waypoints_x[0])        
        self.init_state = np.array([self.waypoints_x[0], self.waypoints_y[0], theta_ref])
        print("initial state: ", self.init_state, self.init_state.shape, self.init_state[0])
        self.export_fig = os.path.join(filepath,name + gaz_bool + '_N'+str(self.N) + '_vref'+str(self.v_ref) + '_T'+str(self.T) + '_vmax'+str(self.v_max) + '_latency'+str(self.latency) + '')
        
        #calculate angle between second to last and last waypoint
        theta_ref = np.arctan2(self.waypoints_y[-1] - self.waypoints_y[-2], self.waypoints_x[-1] - self.waypoints_x[-2])
        self.goal_state = np.array([self.waypoints_x[-1], self.waypoints_y[-1], theta_ref])
        self.Q = np.array([[1.0, 0.0, 0.0],[0.0, 1.0, 0.0],[0.0, 0.0, .5]])*100
        # self.R = np.array([[0.5, 0.0], [0.0, 0.05]])
        self.R = np.array([[2, 0.0], [0.0, 0.0]])
       
        self.opti = ca.Opti()
        # control variables, linear velocity v and angle velocity steer
        self.opt_controls = self.opti.variable(self.N, 2)
        self.v = self.opt_controls[:, 0]
        self.steer = self.opt_controls[:, 1]
        self.opt_states = self.opti.variable(self.N+1, 3)
        self.x = self.opt_states[:, 0]
        self.y = self.opt_states[:, 1]
        self.theta = self.opt_states[:, 2]

        # parameters, these parameters are the reference trajectories of the pose and inputs
        self.opt_u_ref = self.opti.parameter(self.N, 2)
        self.opt_x_ref = self.opti.parameter(self.N+1, 3)

        # bicycle model
        self.f = lambda x_, u_: ca.vertcat(*[u_[0]*ca.cos(x_[2]), u_[0]*ca.sin(x_[2]), u_[0]/self.L*ca.tan(u_[1])])
        self.f_np = lambda x_, u_: np.array([u_[0]*np.cos(x_[2]), u_[0]*np.sin(x_[2]), (u_[0]/self.L)*np.tan(u_[1])])

        ## init_condition
        self.opti.subject_to(self.opt_states[0, :] == self.opt_x_ref[0, :])
        for i in range(self.N):
            x_next = self.opt_states[i, :] + self.f(self.opt_states[i, :], self.opt_controls[i, :]).T*self.T
            self.opti.subject_to(self.opt_states[i+1, :]==x_next)

        self.obstacle = []
        # self.obstacle.append([5, 3, 1.]) # x, y, radius
        for obs in self.obstacle:
            for i in range(self.N+1):
                temp_constraints_ = ca.sqrt((self.opt_states[i, 0]-obs[0])**2+(self.opt_states[i, 1]-obs[1])**2)-self.rob_diam/2.0-obs[2]
                self.opti.subject_to(self.opti.bounded(0.0, temp_constraints_, 10.0))
        
        #### cost function
        self.obj = 0 #### cost
        for i in range(self.N):
            if i == 0:
                self.Qt = np.array([[10.0, 0.0, 0.0],[0.0, 10.0, 0.0],[0.0, 0.0, 5]])  # Larger weight for the closest waypoint
            else:
                self.Qt = self.Q
            state_error_ = self.opt_states[i, :] - self.opt_x_ref[i+1, :]
            control_error_ = self.opt_controls[i, :] - self.opt_u_ref[i, :]
            self.obj = self.obj + ca.mtimes([state_error_, self.Q, state_error_.T]) + ca.mtimes([control_error_, self.R, control_error_.T])

        self.final_state = self.opt_states[-1, :]
        self.opti.minimize(self.obj)

        #### boundrary and control conditions
        self.opti.subject_to(self.opti.bounded(self.x_min, self.x, self.x_max))
        self.opti.subject_to(self.opti.bounded(self.y_min, self.y, self.y_max))
        self.opti.subject_to(self.opti.bounded(-np.pi, self.theta, np.pi))
        self.opti.subject_to(self.opti.bounded(self.v_min, self.v, self.v_max))
        self.opti.subject_to(self.opti.bounded(-self.steer_max, self.steer, self.steer_max))

        self.opts_setting = {'ipopt.max_iter':2000, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}

        self.opti.solver('ipopt', self.opts_setting)

        self.t0 = 0
        self.current_state = self.init_state.copy()
        self.current_state_pp = self.init_state.copy()
        self.u0 = np.zeros((self.N, 2))
        self.next_trajectories = np.tile(self.init_state, self.N+1).reshape(self.N+1, -1) # set the initial state as the first trajectories for the robot
        self.next_controls = np.zeros((self.N, 2))
        self.next_states = np.zeros((self.N+1, 3))
        self.x_c = [] # contains for the history of the state
        self.u_c = []
        self.t_c = [self.t0] # for the time
        self.xx = []
        self.sim_time = 50.0

        ## start MPC
        self.mpciter = 0
        self.start_time = time.time()
        self.index_t = []

    def shift_movement(self, t0, x0, u, x_n, f):
        f_value = f(x0, u[0])
        st = x0 + self.T*f_value
        t = t0 + self.T
        u_end = np.concatenate((u[1:], u[-1:]))
        x_n = np.concatenate((x_n[1:], x_n[-1:]))
        return t, st, u_end, x_n

    def predict_state_with_latency(self, x0, u, latency):
        """
        Predict the state of the system at the end of the latency period.
        """
        num_steps = int(latency / self.T)
        for i in range(num_steps):
            x0 = x0 + self.f_np(x0, u[i]) * self.T
        return x0
    def get_lookahead_point(self, x, y, heading):
        closest_point_idx, closest_point = self.find_closest_waypoint(x, y)
        
        # Search for the lookahead point starting from the closest point
        for i in range(closest_point_idx, len(self.waypoints)):
            if np.linalg.norm(self.waypoints[:,i] - np.array([x, y])) >= self.look_ahead:
                return self.waypoints[:,i]
        # If no lookahead point found within the lookahead distance, take the last waypoint
        return self.waypoints[: -1]
    def get_steering_angle(self, x, y, heading):
        lookahead_point = self.get_lookahead_point(x, y, heading)
        # Compute the angle to the lookahead point in the vehicle coordinate system
        dx = lookahead_point[0] - x
        dy = lookahead_point[1] - y
        angle_to_lookahead = np.arctan2(dy, dx) - heading
        # Keep the angle in the range [-pi, pi]
        while angle_to_lookahead > np.pi:
            angle_to_lookahead -= 2*np.pi
        while angle_to_lookahead < -np.pi:
            angle_to_lookahead += 2*np.pi
        return angle_to_lookahead, lookahead_point[0], lookahead_point[1]
    def find_closest_waypoint(self, x, y):
        distances = np.linalg.norm(np.vstack((self.waypoints_x, self.waypoints_y)).T - np.array([x, y]), axis=1)
        closest_point_idx = np.argmin(distances)
        return closest_point_idx, self.waypoints[:,closest_point_idx]
    
    def find_next_waypoint(self, x, y):
        closest_idx,_ = self.find_closest_waypoint(x, y)
        # Define a region of acceptance around the waypoint (change the value as needed)
        region_of_acceptance = 0.05
        # Check if the vehicle is within the region of acceptance for the closest waypoint
        dist_to_waypoint = np.linalg.norm([x - self.waypoints_x[closest_idx], y - self.waypoints_y[closest_idx]])
        if dist_to_waypoint < region_of_acceptance:
            target_idx = min(closest_idx + 1, len(self.waypoints_x) - 1)
        else:
            target_idx = closest_idx
        return target_idx

    def desired_trajectory(self, index):
        x_ref = self.waypoints_x[index]
        y_ref = self.waypoints_y[index]

        # Calculate the orientation (theta) based on the change in x and y between consecutive waypoints
        if index < len(self.waypoints_x) - 1:
            theta_ref = np.arctan2(self.waypoints_y[index+1] - y_ref, self.waypoints_x[index+1] - x_ref)
        else: # For the last point, use the previous point to calculate theta
            theta_ref = np.arctan2(y_ref - self.waypoints_y[index-1], x_ref - self.waypoints_x[index-1])

        steer_ref = 0.0 

        return x_ref, y_ref, theta_ref, self.v_ref, steer_ref

    def desired_command_and_trajectory(self, index, N_):
        # Get a series of reference states and inputs based on the index.
        # Create arrays to hold the reference states and inputs
        x_refs = np.zeros(N_ + 1)
        y_refs = np.zeros(N_ + 1)
        theta_refs = np.zeros(N_ + 1)
        v_refs = np.zeros(N_)
        steer_refs = np.zeros(N_)

        # Populate the reference states and inputs based on the waypoints
        for i in range(N_):
            if index + i < len(self.waypoints_x) - 1:
                x_refs[i], y_refs[i], theta_refs[i], v_refs[i], steer_refs[i] = self.desired_trajectory(index + i)
            else: # If we have reached the end of the waypoints, just use the last waypoint for the remaining steps
                x_refs[i], y_refs[i], theta_refs[i], v_refs[i], steer_refs[i] = self.desired_trajectory(len(self.waypoints_x) - 1)
        
        # Get the last reference state separately (without setting v_ref and steer_ref)
        if index + N_ < len(self.waypoints_x) - 1:
            x_refs[N_], y_refs[N_], theta_refs[N_] = self.waypoints_x[index + N_], self.waypoints_y[index + N_], theta_refs[N_ - 1]
        else:
            x_refs[N_], y_refs[N_], theta_refs[N_] = self.waypoints_x[-1], self.waypoints_y[-1], theta_refs[N_ - 1]

        # Combine x, y, and theta references into a single array for the state references
        state_refs = np.vstack((x_refs, y_refs, theta_refs)).T

        # Combine v and steer references into a single array for the input references
        input_refs = np.vstack((v_refs, steer_refs)).T

        return state_refs, input_refs

    def model_callback(self, model):
        if self.car_idx is None:
            try:
                self.car_idx = model.name.index("automobile")
            except ValueError:
                return
        car_pose = model.pose[self.car_idx]
        with self.lock:
            self.real_x = car_pose.position.x
            self.real_y = car_pose.position.y+15

    def imu_callback(self, imu):
        with self.lock:
            self.yaw = imu.yaw

if __name__ == '__main__':
    mpc = MPC()
    if mpc.gazebo:
        def spin_thread():
            rospy.spin()
        thread = threading.Thread(target=spin_thread)
        thread.start()
        rate = rospy.Rate(1/mpc.T)
    # wait until the first state is received
    while mpc.gazebo and (mpc.real_x is None or mpc.real_y is None or mpc.yaw is None):
        pass
    if mpc.gazebo:
        print("starting... x: ", mpc.real_x, ", y: ", mpc.real_y, ", yaw: ", mpc.yaw)
    # stop when last waypoint is reached
    length = len(mpc.waypoints_x)
    target_waypoint_index = 0
    print("length: ", length)
    while(True):
        if target_waypoint_index >= length-2:
            break
        t = time.time()
        steering_angle, dx, dy = mpc.get_steering_angle(mpc.real_x, mpc.real_y, mpc.yaw)
        #clip 
        steering_angle = np.clip(steering_angle*180/np.pi, -23, 23)
        # print("steering_angle: ", steering_angle, dx, dy, mpc.real_x, mpc.real_y)
        # print with 2 digits after the decimal point
        print("i) ", mpc.mpciter, "dx: ", np.around(dx, decimals=2), ", dy: ", np.around(dy, decimals=2), ", x: ",
               np.around(mpc.real_x, decimals=2), ", y: ", np.around(mpc.real_y, decimals=2), "steering_angle: ", steering_angle)
        if mpc.gazebo:
            mpc.msg.data = '{"action":"1","speed":'+str(0.05)+'}'
            mpc.pub.publish(mpc.msg)
            mpc.msg.data = '{"action":"2","steerAngle":'+str(float(steering_angle))+'}'
            mpc.pub.publish(mpc.msg)
            rate.sleep()
        # print("i) ", mpc.mpciter, "x: ", np.around(mpc.current_state[0], decimals=2), ", y: ",
        #       np.around(mpc.current_state[1], decimals=2), ", yaw: ",np.around(mpc.current_state[2], decimals=2),
        #         "u: ", np.around(u_res[0, :], decimals=2), "time: ", time.time()-t, "index: ", target_waypoint_index)
        # print("index: ", target_waypoint_index)
    if mpc.gazebo:
        thread.join()
    ## after loop
    print(mpc.mpciter)
    t_v = np.array(mpc.index_t)
    print(t_v.mean())
    print((time.time() - mpc.start_time)/(mpc.mpciter))
    # print the control input, keep 2 digits after the decimal point
    mpc.u_c = np.array(mpc.u_c)
    mpc.x_c = np.array(mpc.x_c)
    np.save('mpc_u_c.npy', mpc.u_c)
    print("u: ", np.around(mpc.u_c, decimals=2))
    print("x: ", np.around(mpc.x_c, decimals=2))
    ## draw function
    draw_result = Draw_MPC_tracking(grid = None, obstacle = mpc.obstacle, rob_diam=0.3, init_state=mpc.init_state, 
                                    robot_states=mpc.xx, export_fig=mpc.export_fig, waypoints_x=mpc.waypoints_x, 
                                    waypoints_y=mpc.waypoints_y)
