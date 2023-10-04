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
from scipy.interpolate import interp1d
from scipy.interpolate import splprep, splev
import argparse

def filter_waypoints(waypoints, threshold):
    filtered_waypoints = [waypoints[0]]
    
    for i in range(1, len(waypoints)):
        if np.linalg.norm(np.array(waypoints[i]) - np.array(waypoints[i-1])) >= threshold:
            filtered_waypoints.append(waypoints[i])
        else: 
            print("filtered out waypoint: ",i, waypoints[i])
    
    return np.array(filtered_waypoints)

def interpolate_waypoints(waypoints, num_points):
    x = waypoints[:, 0]
    y = waypoints[:, 1]

    tck, u = splprep([x, y], s=0) 

    # Generate set of equally spaced waypoints
    u_new = np.linspace(0, 1, num_points)
    x_new, y_new = splev(u_new, tck) 

    # Stack the x and y coordinates to get new waypoints
    new_waypoints = np.vstack((x_new, y_new)).T

    return new_waypoints

def densify_waypoints(waypoints, num_points):
    x = waypoints[:, 0]
    y = waypoints[:, 1]
    i = np.arange(len(waypoints))

    # Create interpolation functions for x and y coordinates
    interp_x = interp1d(i, x, kind='linear')
    interp_y = interp1d(i, y, kind='linear')

    i_new = np.linspace(0, len(waypoints) - 1, num_points)

    # Get the interpolated x and y coordinates
    x_new = interp_x(i_new)
    y_new = interp_y(i_new)

    new_waypoints = np.vstack((x_new, y_new)).T
    return new_waypoints

class MPC:
    def __init__(self, gazebo):
        self.gazebo = gazebo
        gaz_bool = ""
        if self.gazebo:
            gaz_bool = "_gazebo_"
            rospy.init_node("mpc_node")
            self.pub = rospy.Publisher("automobile/command",String,queue_size=3)
            self.msg = String()  
            self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback, queue_size=3)
            self.imu_sub = rospy.Subscriber("/automobile/IMU", IMU, self.imu_callback, queue_size=3)
            self.real_x = None
            self.real_y = None
            self.yaw = None
            self.car_idx = None
            self.lock = threading.Lock()
        name = 'speedrun'
        density = 4
        self.last_waypoint_index = 1
        filepath = os.path.dirname(os.path.abspath(__file__))
        speedrun2 = np.load(os.path.join(filepath, 'waypoints/speedrun2_noint.npy'))
        speedrun3 = np.load(os.path.join(filepath,'waypoints/speedrun3_noint.npy'))
        speedrun4 = np.load(os.path.join(filepath,'waypoints/speedrun4_noint.npy'))
        speedrun1 = np.load(os.path.join(filepath,'waypoints/speedrun1_noint.npy'))
        print(speedrun1.shape, speedrun2.shape, speedrun3.shape, speedrun4.shape)
        # delete third waypoint
        # speedrun1 = np.delete(speedrun1, 2, 1)
        self.waypoints1 = interpolate_waypoints(speedrun1.T, int(len(speedrun1[0,:])*density))
        self.waypoints2 = interpolate_waypoints(speedrun2.T, int(len(speedrun2[0,:])*density))
        self.waypoints3 = interpolate_waypoints(speedrun3.T, int(len(speedrun3[0,:])*density))
        self.waypoints4 = interpolate_waypoints(speedrun4.T, int(len(speedrun4[0,:])*density))
        print(self.waypoints1.shape, self.waypoints2.shape, self.waypoints3.shape, self.waypoints4.shape)
        self.waypoints = np.vstack((self.waypoints1, self.waypoints2, self.waypoints3, self.waypoints4))
        print("waypoints: ", self.waypoints.shape)
        print(self.waypoints)
        self.waypoints = filter_waypoints(self.waypoints, 0.01).T
        #calculate the path length of the waypoints
        path_length = 0
        for i in range(len(self.waypoints[0])-1):
            path_length += np.linalg.norm(self.waypoints[:,i+1]-self.waypoints[:,i])
        print("path length: ", path_length)
        # self.waypoints = self.waypoints[:,:280]
        # self.waypoints = self.waypoints[:,280:]
        # self.waypoints[0] += 0.5
        # self.waypoints[1] -= 1.5
        # self.waypoints = np.vstack((self.waypoints[1], self.waypoints[0])) #flip x and y
        # self.waypoints[0] = 15-self.waypoints[0]
        # self.waypoints[1] = 15-self.waypoints[1]
        # self.waypoints = self.waypoints[:, 0:24]
        # self.waypoints = self.waypoints[:, :15]
        self.waypoints_x = self.waypoints[0, :]
        self.waypoints_y = self.waypoints[1, :]
        self.x_max = 15
        self.x_min = 0
        self.y_max = 15
        self.y_min = 0
        print("x_max: ", self.x_max, ", x_min: ", self.x_min, ", y_max: ", self.y_max, ", y_min: ", self.y_min)
        # Constants
        self.L = 0.27
        self.latency = 0#.7245
        self.T = 0.2415
        self.N = 8
        self.rob_diam = 0.3
        self.v_max = 0.753*2
        self.v_min = -0.753
        self.steer_max = 0.4003
        self.v_ref = 0.537*2
        self.xy_cost = 1
        self.yaw_cost = 0.5
        self.v_cost = 2 
        self.steer_cost = 0.25
        self.steer_ref = 0.0
        #compute angle between first and second waypoint
        start_index = 57
        theta_ref = np.arctan2(self.waypoints_y[start_index+1] - self.waypoints_y[start_index],
                                self.waypoints_x[start_index+1] - self.waypoints_x[start_index])        
        self.init_state = np.array([self.waypoints_x[start_index], self.waypoints_y[start_index], theta_ref])
        index, _ = self.find_closest_waypoint(self.init_state[0], self.init_state[1])
        # delete waypoints before the initial state
        self.waypoints_x = self.waypoints_x[index:]
        self.waypoints_y = self.waypoints_y[index:]
        print("initial state: ", self.init_state, self.init_state.shape, self.init_state[0])
        self.export_fig = os.path.join(filepath,name + gaz_bool + '_N'+str(self.N) + '_vref'+str(self.v_ref) 
                                       + '_T'+str(self.T) + '_vmax'+str(self.v_max) + '_latency'+str(self.latency) 
                                       + 'costs'+str(self.xy_cost)+'_'+str(self.yaw_cost)+'_'+str(self.v_cost)+'_'+
                                       str(self.steer_cost)+'density'+str(density)+'')
        
        self.Q = np.array([[self.xy_cost, 0.0, 0.0],[0.0, self.xy_cost, 0.0],[0.0, 0.0, self.yaw_cost]])*1
        self.R = np.array([[self.v_cost, 0.0], [0.0, self.steer_cost]])*1
       
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
        # self.obstacle.append([4, 2, 0.3]) # x, y, radius
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
            yaw_error = self.opt_states[i, 2] - self.opt_x_ref[i+1, 2]
            yaw_error = ca.atan2(ca.sin(yaw_error), ca.cos(yaw_error))
            state_error_ = ca.vertcat(self.opt_states[i, 0] - self.opt_x_ref[i+1, 0], self.opt_states[i, 1] - self.opt_x_ref[i+1, 1], yaw_error)
            state_error_ = state_error_.T
            # state_error_ = self.opt_states[i, :] - self.opt_x_ref[i+1, :]
            control_error_ = self.opt_controls[i, :] - self.opt_u_ref[i, :]
            self.obj = self.obj + ca.mtimes([state_error_, self.Q, state_error_.T]) + ca.mtimes([control_error_, self.R, control_error_.T])

        self.final_state = self.opt_states[-1, :]
        self.opti.minimize(self.obj)

        #### boundrary and control conditions
        self.opti.subject_to(self.opti.bounded(self.x_min, self.x, self.x_max))
        self.opti.subject_to(self.opti.bounded(self.y_min, self.y, self.y_max))
        self.opti.subject_to(self.opti.bounded(-np.pi*3, self.theta, np.pi*3))
        self.opti.subject_to(self.opti.bounded(self.v_min, self.v, self.v_max))
        self.opti.subject_to(self.opti.bounded(-self.steer_max, self.steer, self.steer_max))

        self.opts_setting = {'ipopt.max_iter':2000, 'ipopt.print_level':0, 'print_time':0, 'ipopt.acceptable_tol':1e-8, 'ipopt.acceptable_obj_change_tol':1e-6}

        self.opti.solver('ipopt', self.opts_setting)

        self.t0 = 0
        self.current_state = self.init_state.copy()
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
        num_steps = int(latency / self.T)
        print("num_steps: ", num_steps)
        for i in range(num_steps):
            x0 = x0 + self.f_np(x0, u[i]) * self.T
        return x0
    
    def find_closest_waypoint(self, x, y):
        distances = np.linalg.norm(np.vstack((self.waypoints_x, self.waypoints_y)).T - np.array([x, y]), axis=1)
        index = np.argmin(distances)
        return index, distances[index]
    
    def find_next_waypoint(self, x, y):
        closest_idx, dist_to_waypoint = self.find_closest_waypoint(x, y)
        region_of_acceptance = 0.05
    
        # Ensure we are moving forward in the waypoint list, but not jumping too far ahead
        if dist_to_waypoint < region_of_acceptance:
            if closest_idx - self.last_waypoint_index < 15:
                self.last_waypoint_index = max(self.last_waypoint_index, closest_idx)
            else:
                print("here: ", closest_idx, self.last_waypoint_index, closest_idx - self.last_waypoint_index)
                closest_idx = self.last_waypoint_index + 1
                # self.last_waypoint_index += 1
        else:
            if closest_idx - self.last_waypoint_index > 15:
                print("here2: ", closest_idx, self.last_waypoint_index, closest_idx - self.last_waypoint_index)
                closest_idx = self.last_waypoint_index + 1
            # If not within the region of acceptance, take smaller steps forward in the waypoint list
            self.last_waypoint_index += 1

        target_idx = max(self.last_waypoint_index, closest_idx)
        return min(target_idx, len(self.waypoints_x) - 1)

    def desired_trajectory(self, index):
        x_ref = self.waypoints_x[index]
        y_ref = self.waypoints_y[index]

        # Calculate the orientation (theta) based on the change in x and y between consecutive waypoints
        if index < len(self.waypoints_x) - 1:
            theta_ref = np.arctan2(self.waypoints_y[index+1] - y_ref, self.waypoints_x[index+1] - x_ref)
        else: # For the last point, use the previous point to calculate theta
            theta_ref = np.arctan2(y_ref - self.waypoints_y[index-1], x_ref - self.waypoints_x[index-1])

        # print("index: ", index, "x_ref: ", np.around(x_ref, decimals=2), "y_ref: ", np.around(y_ref, decimals=2), "theta_ref: ", np.around(theta_ref, decimals=2))
        return x_ref, y_ref, theta_ref, self.v_ref, self.steer_ref

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
    parser = argparse.ArgumentParser()
    parser.add_argument('--gazebo', action='store_true', help='Boolean for whether the simulation is in gazebo or not')

    mpc = MPC(gazebo=parser.parse_args().gazebo)

    if mpc.gazebo:
        print("gazebo!")
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
        # if target_waypoint_index >= length-2:
        if target_waypoint_index >= 345:
            break
        t = time.time()
        if mpc.gazebo:
            with mpc.lock:
                mpc.current_state[0] = mpc.real_x
                mpc.current_state[1] = mpc.real_y
                mpc.current_state[2] = mpc.yaw
        initial_state_with_latency = mpc.predict_state_with_latency(mpc.current_state, mpc.next_controls, mpc.latency)
        mpc.opti.set_value(mpc.opt_x_ref[0, :], initial_state_with_latency)
        
        target_waypoint_index = mpc.find_next_waypoint(mpc.current_state[0], mpc.current_state[1])
        # make sure yaw between -pi and pi
        mpc.current_state[2] = np.arctan2(np.sin(mpc.current_state[2]), np.cos(mpc.current_state[2]))
        mpc.next_trajectories, mpc.next_controls = mpc.desired_command_and_trajectory(target_waypoint_index, mpc.N)
        ## set parameter, here only update initial state of x (x0)
        mpc.opti.set_value(mpc.opt_x_ref[1:,:], mpc.next_trajectories[1:,:])
        mpc.opti.set_value(mpc.opt_u_ref, mpc.next_controls)
        ## provide the initial guess of the optimization targets
        mpc.opti.set_initial(mpc.opt_controls, mpc.u0.reshape(mpc.N, 2))# (N, 2)
        mpc.opti.set_initial(mpc.opt_states, mpc.next_states) # (N+1, 3)
        ## solve the problem once again
        t_ = time.time()
        sol = mpc.opti.solve()
        mpc.index_t.append(time.time()- t_)
        ## obtain the control input
        u_res = sol.value(mpc.opt_controls)
        x_m = sol.value(mpc.opt_states)
        if target_waypoint_index < length-1:
            print( "ref_traj:", np.around(mpc.next_trajectories, decimals=2)[0], 
                "cur:", np.around(mpc.current_state, decimals=2), "ctrl:", 
                np.around(u_res[0, :], decimals=2), "idx:", target_waypoint_index)
        mpc.u_c.append(u_res[0, :])
        mpc.t_c.append(mpc.t0)
        mpc.x_c.append(x_m)
        mpc.t0, mpc.current_state, mpc.u0, next_states = mpc.shift_movement(mpc.t0, mpc.current_state, u_res, x_m, mpc.f_np)
        mpc.xx.append(mpc.current_state)
        mpc.mpciter = mpc.mpciter + 1
        if mpc.gazebo:
            mpc.msg.data = '{"action":"1","speed":'+str(u_res[0, 0])+'}'
            mpc.pub.publish(mpc.msg)
            mpc.msg.data = '{"action":"2","steerAngle":'+str(-float(u_res[0, 1]*180/np.pi))+'}'
            mpc.pub.publish(mpc.msg)
            rate.sleep()
        if mpc.gazebo:
            print("i) ", mpc.mpciter, "x: ", np.around(mpc.current_state[0], decimals=2), ", y: ",
                np.around(mpc.current_state[1], decimals=2), ", yaw: ",np.around(mpc.current_state[2], decimals=2),
                    "u: ", np.around(u_res[0, :], decimals=2), "time: ", time.time()-t, "index: ", target_waypoint_index)
        # print("index: ", target_waypoint_index)
    if mpc.gazebo:
        mpc.msg.data = '{"action":"1","speed":'+str(0.0)+'}'
        mpc.pub.publish(mpc.msg)
        mpc.msg.data = '{"action":"2","steerAngle":'+str(0.0)+'}'
        mpc.pub.publish(mpc.msg)
        thread.join()
    ## after loop
    print(mpc.mpciter)
    t_v = np.array(mpc.index_t)
    print(t_v.mean())
    print((time.time() - mpc.start_time)/(mpc.mpciter))
    # print the control input, keep 2 digits after the decimal point
    mpc.u_c = np.array(mpc.u_c)
    mpc.x_c = np.array(mpc.x_c)
    print("average speed: ",  np.mean(mpc.u_c, axis=0)[0])
    # np.save('mpc_u_c.npy', mpc.u_c)
    # print("u: ", np.around(mpc.u_c, decimals=2))
    # print("x: ", np.around(mpc.x_c, decimals=2))
    ## draw function
    draw_result = Draw_MPC_tracking(obstacle = mpc.obstacle, rob_diam=0.3, init_state=mpc.init_state, 
                                    robot_states=mpc.xx, export_fig=mpc.export_fig, waypoints_x=mpc.waypoints_x, 
                                    waypoints_y=mpc.waypoints_y)
