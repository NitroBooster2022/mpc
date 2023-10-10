#!/usr/bin/env python3

import time
from draw import Draw_MPC_tracking
import casadi as ca
import numpy as np
import rospy
from std_msgs.msg import String, Float32
from gazebo_msgs.msg import ModelStates
from utils.msg import IMU
import threading
import os
from sensor_msgs.msg import Imu
from scipy.interpolate import interp1d, UnivariateSpline, splprep, splev
import argparse
import tf
from utility import Utility
from std_srvs.srv import Trigger
from global_planner import GlobalPlanner

def call_trigger_service():
    try:
        trigger_service = rospy.ServiceProxy('trigger_service', Trigger)
        response = trigger_service()
        rospy.loginfo("Service response: %s", response.message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
def compute_smooth_curvature(waypoints_x, waypoints_y, smooth_factor=0.1):
    # Step 1: Set up a spline interpolation
    t = np.linspace(0, 1, len(waypoints_x))
    spline_x = UnivariateSpline(t, waypoints_x, k=3, s=smooth_factor)
    spline_y = UnivariateSpline(t, waypoints_y, k=3, s=smooth_factor)

    # Step 2: Get smoothed derivatives of the path
    t_smooth = np.linspace(0, 1, len(waypoints_x))
    dx_dt = spline_x.derivative()(t_smooth)
    dy_dt = spline_y.derivative()(t_smooth)
    
    ddx_dt = spline_x.derivative(n=2)(t_smooth)
    ddy_dt = spline_y.derivative(n=2)(t_smooth)
    
    # Step 3: Compute curvature
    # κ = |dx/dt * d²y/dt² - dy/dt * d²x/dt²| / (dx/dt² + dy/dt²)^(3/2)
    curvature = np.abs(dx_dt * ddy_dt - dy_dt * ddx_dt) / (dx_dt**2 + dy_dt**2)**(3/2)
    
    # Step 4: Compute tangent 
    tangent_angles = np.arctan2(dy_dt, dx_dt)

    # Step 5: Compute normal
    normal_angles = tangent_angles + np.pi / 2
    # Convert normal angles to vectors (dx, dy)
    dx = np.cos(normal_angles)
    dy = np.sin(normal_angles)
    normals = np.vstack((dx, dy)).T
    
    return curvature, tangent_angles, normals

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
    def __init__(self, gazebo, useIMU=False, useEkf=False, sign=False):
        self.gazebo = gazebo
        gaz_bool = ""
        if self.gazebo:
            gaz_bool = "_gazebo_"
            rospy.init_node("mpc_node")
            self.useIMU = useIMU
            self.utils = Utility(useIMU=useIMU, subLane=False, subImu=True, subModel=True, pubOdom=True, useEkf=useEkf, subSign=sign)
            self.real_x = None
            self.real_y = None
            self.yaw = None
            self.lock = threading.Lock()
        self.x_max = 15
        self.x_min = 0
        self.y_max = 15
        self.y_min = 0
        # Constants
        self.L = 0.27
        self.latency = 0.#7245/3
        self.T = 0.2415 #simulation time step
        self.latency_num = int(self.latency/self.T)
        self.region_of_acceptance = 0.05
        self.N = 10 # prediction horizon
        self.rob_diam = 0.3
        self.v_max = 0.753*1
        self.v_min = -0.753
        self.steer_max = 0.400553
        self.v_ref = 0.753*1
        self.steer_ref = 0.0
        self.xy_cost = 1#*2
        self.yaw_cost = 0#.5
        self.v_cost = 1 
        self.steer_cost = 0.05#.25
        self.delta_v_cost = 0.25
        self.delta_steer_cost = 0.5
        self.costs = np.array([self.xy_cost, self.yaw_cost, self.v_cost, self.steer_cost, self.delta_v_cost, self.delta_steer_cost])
        
        self.global_planner = GlobalPlanner()
        run1, _ = self.global_planner.plan_path(86, 467)
        run2, _ = self.global_planner.plan_path(467, 302)
        run3, _ = self.global_planner.plan_path(302, 264)
        run4, _ = self.global_planner.plan_path(264, 81)
        runs = [run1, run2, run3, run4]
        # runs = [np.load(os.path.join(filepath, f'waypoints/speedrun{i}_noint.npy')) for i in range(1, 5)]
        start = self.global_planner.place_names['start']
        dest1 = self.global_planner.place_names['highway1_start']
        dest2 = self.global_planner.place_names['highway1_mid']
        dest3 = self.global_planner.place_names['highway2_start']
        dest4 = self.global_planner.place_names['highway2_mid']
        dest5 = self.global_planner.place_names['parallel_parking1']
        run1, _ = self.global_planner.plan_path(start, dest1)
        run2, _ = self.global_planner.plan_path(dest1, dest2)
        run3, _ = self.global_planner.plan_path(dest2, dest3)
        run4, _ = self.global_planner.plan_path(dest3, dest4)
        run5, _ = self.global_planner.plan_path(dest4, dest5)
        runs = [run1, run2]#, run3, run4, run5]
        name = 'speedrun'
        self.last_waypoint_index = 1
        filepath = os.path.dirname(os.path.abspath(__file__))
        # wpts = np.array([[2.37, 13.3],[2.5, 13.32],[2.8, 13.36], [3.214,13.644]]).T
        # runs.append(wpts)
        # print("shape2: ",wpts.shape)

        # Compute path lengths 
        path_lengths = [np.sum(np.linalg.norm(run[:, 1:] - run[:, :-1], axis=0)) for run in runs]
        for i, length in enumerate(path_lengths):
            print(i, ") path length: ", length)
            runs[i] = interpolate_waypoints(runs[i].T, int(np.ceil(length*10)))
        # Combine all runs into a single set of waypoints
        self.waypoints = np.vstack(runs)
        print("waypoints: ", self.waypoints.shape)
        self.waypoints = filter_waypoints(self.waypoints, 0.01).T
        # Calculate the total path length of the waypoints
        total_path_length = np.sum(np.linalg.norm(self.waypoints[:, 1:] - self.waypoints[:, :-1], axis=0))
        print("path length: ", total_path_length)
        # self.waypoints = self.waypoints[:,:280]
        # self.waypoints = self.waypoints[:,280:]
        # self.waypoints[0] += 0.5
        # self.waypoints[1] -= 1.5
        # self.waypoints = np.vstack((self.waypoints[1], self.waypoints[0])) #flip x and y
        # self.waypoints[0] = 15-self.waypoints[0] # flip x
        # self.waypoints[1] = 15-self.waypoints[1]
        # self.waypoints = self.waypoints[:, 0:24]
        # self.waypoints = self.waypoints[:, :15]
        self.waypoints_x = self.waypoints[0, :]
        self.waypoints_y = self.waypoints[1, :]
        # wpts = np.load(os.path.join(filepath, 'waypoints/parallel_park.npy'))
        # self.waypoints_x = wpts[:, 0]+5
        # self.waypoints_y = wpts[:, 1]+5
        start_index = int(len(self.waypoints_x)*0)
        end_index = int(len(self.waypoints_x)*1)
        theta_ref = np.arctan2(self.waypoints_y[start_index+1] - self.waypoints_y[start_index],
                                self.waypoints_x[start_index+1] - self.waypoints_x[start_index])        
        self.init_state = np.array([self.waypoints_x[start_index], self.waypoints_y[start_index], theta_ref])
        # self.init_state = np.array([0.82, 0.09, 3.14159/2])
        index, _ = self.find_closest_waypoint(self.init_state[0], self.init_state[1])
        self.waypoints_x = self.waypoints_x[index:end_index]
        self.waypoints_y = self.waypoints_y[index:end_index]
        self.num_waypoints = len(self.waypoints_x)
        self.kappa, self.wp_theta, self.wp_normals = compute_smooth_curvature(self.waypoints_x, self.waypoints_y)
        self.v_refs  = self.v_ref / (1 + np.abs(self.kappa))
        self.v_refs[-2:] = 0
        k_steer = 0 #0.4/np.amax(np.abs(self.kappa))
        self.steer_ref = k_steer * self.kappa
        # Extend waypoints and reference values by N
        self.waypoints_x = np.pad(self.waypoints_x, (0,self.N+4), 'edge')
        self.waypoints_y = np.pad(self.waypoints_y, (0,self.N+4), 'edge')
        self.kappa = np.pad(self.kappa, (0,self.N+5), 'edge')
        self.wp_theta = np.pad(self.wp_theta, (0,self.N+5), 'edge')
        self.wp_normals = np.pad(self.wp_normals, ((0,self.N+5),(0,0)), 'edge')
        self.v_refs = np.pad(self.v_refs, (0,self.N+5), 'edge')
        self.steer_ref = np.pad(self.steer_ref, (0,self.N+5), 'edge')
        self.state_refs = np.vstack((self.waypoints_x, self.waypoints_y, self.wp_theta[1:])).T
        self.input_refs = np.vstack((self.v_refs, self.steer_ref)).T
        self.waypoints = np.vstack((self.waypoints_x, self.waypoints_y)).T
        print("state_refs: ", self.state_refs.shape, ", input_refs: ", self.input_refs.shape)

        # self.change_lane(589, 609, self.wp_normals)
        # self.change_lane(616, 636, self.wp_normals)
        print("initial state: ", self.init_state, self.init_state.shape, self.init_state[0])
        
        self.export_fig = os.path.join(filepath+'/gifs',name + gaz_bool + '_N'+str(self.N) + '_vref'+str(self.v_ref) 
                                       + '_T'+str(self.T) + '_vmax'+str(self.v_max) + '_latency'+str(self.latency) 
                                       + 'costs'+str(self.xy_cost)+'_'+str(self.yaw_cost)+'_'+str(self.v_cost)+'_'+
                                       str(self.steer_cost)+'_'+str(self.delta_v_cost)+'_'+str(self.delta_steer_cost)+'')
        
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
            # x_next[0,2] = ca.atan2(ca.sin(x_next[0,2]), ca.cos(x_next[0,2]))
            self.opti.subject_to(self.opt_states[i+1, :]==x_next)

        self.obstacle = []
        # self.obstacle.append([4, 2, 0.3]) # x, y, radius
        # for obs in self.obstacle:
        #     for i in range(self.N+1):
        #         temp_constraints_ = ca.sqrt((self.opt_states[i, 0]-obs[0])**2+(self.opt_states[i, 1]-obs[1])**2)-self.rob_diam/2.0-obs[2]
        #         self.opti.subject_to(self.opti.bounded(0.0, temp_constraints_, 10.0))
        
        #### cost function
        self.obj = 0 #### cost
        for i in range(self.N):
            # yaw = ca.atan2(ca.sin(self.opt_states[i, 2]), ca.cos(self.opt_states[i, 2]))
            # yaw_ref = ca.atan2(ca.sin(self.opt_x_ref[i+1, 2]), ca.cos(self.opt_x_ref[i+1, 2]))
            # yaw_error = yaw - yaw_ref
            yaw_error = self.opt_states[i, 2] - self.opt_x_ref[i+1, 2]
            yaw_error = ca.atan2(ca.sin(yaw_error), ca.cos(yaw_error))
            state_error_ = ca.vertcat(self.opt_states[i, 0] - self.opt_x_ref[i+1, 0], self.opt_states[i, 1] - self.opt_x_ref[i+1, 1], yaw_error)
            state_error_ = state_error_.T
            # state_error_ = self.opt_states[i, :] - self.opt_x_ref[i+1, :]
            control_error_ = self.opt_controls[i, :] - self.opt_u_ref[i, :]
            if i > 0:
                delta_v = self.opt_controls[i, 0] - self.opt_controls[i - 1, 0]
                delta_steer = self.opt_controls[i, 1] - self.opt_controls[i - 1, 1]
                self.obj += self.delta_v_cost * delta_v**2 + self.delta_steer_cost * delta_steer**2
            self.obj = self.obj + ca.mtimes([state_error_, self.Q, state_error_.T]) + ca.mtimes([control_error_, self.R, control_error_.T])

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
        for i in range(self.latency_num):
            self.u_c.append(np.zeros(2))
        self.u_cc = []
        for i in range(self.latency_num):
            self.u_cc.append(np.zeros((self.N,2)))
        self.t_c = [self.t0] # for the time
        self.xx = []
        self.x_refs = []
        self.x_errors = []
        self.y_errors = []
        self.yaw_errors = []
        ## start MPC
        self.mpciter = 0
        self.start_time = time.time()
        self.index_t = []
    def change_lane(self, start_index, end_index, normals, shift_distance=0.36):
        """
        Shift waypoints in the range [start_index, end_index] by shift_distance meters in the direction of their normal.
        
        Args:
        - start_index: Starting index of waypoints to shift.
        - end_index: Ending index of waypoints to shift.
        - shift_distance: Distance to shift waypoints by.
        """
        self.state_refs[start_index:end_index,0] += normals[start_index:end_index, 0] * shift_distance
        self.state_refs[start_index:end_index,1] += normals[start_index:end_index, 1] * shift_distance
    def shift_movement(self, t0, x0, u, x_n, f): 
        f_value = f(x0, u[0])
        st = x0 + self.T*f_value
        t = t0 + self.T
        u_end = np.concatenate((u[1:], u[-1:]))
        x_n = np.concatenate((x_n[1:], x_n[-1:]))
        return t, st, u_end, x_n
    
    def add_gaussian_noise(self, state):
        state_noise_std_dev = np.array([
            0.02, 
            0.02, 
            1*np.pi/180
        ])
        state_noise = np.random.normal(0, state_noise_std_dev, mpc.current_state.shape)
        return state + state_noise

    def predict_state_with_latency(self, x0):
        for i in range(self.latency_num):
            x0 = x0 + self.f_np(x0, self.u_c[-self.latency_num+i]) * self.T
        return x0
    
    def find_closest_waypoint(self, x, y):
        distances = np.linalg.norm(np.vstack((self.waypoints_x, self.waypoints_y)).T - np.array([x, y]), axis=1)
        index = np.argmin(distances)
        return index, distances[index]
    
    def find_next_waypoint(self, x, y):
        closest_idx, dist_to_waypoint = self.find_closest_waypoint(x, y)
    
        # Ensure we are moving forward in the waypoint list, but not jumping too far ahead
        if dist_to_waypoint < self.region_of_acceptance:
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

    def desired_command_and_trajectory(self, index):
        return self.state_refs[index:index + self.N + 1], self.input_refs[index:index + self.N]

    def update_and_solve(self):
        ## set parameter, here only update initial state of x (x0)
        self.opti.set_value(self.opt_x_ref[1:,:], self.next_trajectories[1:])
        self.opti.set_value(self.opt_u_ref, self.next_controls)
        ## provide the initial guess of the optimization targets
        self.opti.set_initial(self.opt_controls, self.u0.reshape(self.N, 2))
        self.opti.set_initial(self.opt_states, self.next_states)
        ## solve the problem once again
        sol = self.opti.solve()
        return sol.value(self.opt_controls), sol.value(self.opt_states)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--gazebo', action='store_true', help='Boolean for whether the simulation is in gazebo or not')
    parser.add_argument('--useEkf', action='store_true', help='Boolean for whether to use ekf node or not')
    parser.add_argument('--odom', action='store_true', help='Boolean for whether to use odometry or gps data')
    parser.add_argument('--sign', action='store_true', help='Boolean for whether to use sign or not')
    args = parser.parse_args()
    mpc = MPC(gazebo=args.gazebo, useEkf=args.useEkf, sign=args.sign)

    if mpc.gazebo:
        print("gazebo!")
        def spin_thread():
            rospy.spin()
        thread = threading.Thread(target=spin_thread)
        thread.start()
        rate = rospy.Rate(1/mpc.T)
    # wait until the first state is received
    print("waiting for first state...")
    while mpc.gazebo and not mpc.utils.initializationFlag:
        time.sleep(0.1)
    if mpc.gazebo:
        mpc.real_x = mpc.utils.gps_x
        mpc.real_y = mpc.utils.gps_y
        mpc.yaw = mpc.utils.yaw
    while mpc.gazebo and (mpc.real_x is None or mpc.real_y is None or mpc.yaw is None):
        pass
    if mpc.gazebo:
        print("starting... x: ", mpc.real_x, ", y: ", mpc.real_y, ", yaw: ", mpc.yaw)
        mpc.detected_index = 0
    # stop when last waypoint is reached
    length = len(mpc.waypoints_x)
    target_waypoint_index = 0
    print("length: ", length)
    while True:
        if target_waypoint_index >= mpc.num_waypoints-1:
        # if target_waypoint_index >= 375:
            break
        t = time.time()
        if mpc.gazebo:
            if args.sign:
                if mpc.utils.object_detected(12) and target_waypoint_index>=mpc.detected_index:
                    print("car detected!, changing lane...")
                    mpc.detected_index = target_waypoint_index+20
                    mpc.change_lane(target_waypoint_index, mpc.detected_index, mpc.wp_normals)
            with mpc.lock:
                mpc.current_state[2] = mpc.utils.yaw
                if args.odom:
                    mpc.current_state[0] = mpc.utils.odomX
                    mpc.current_state[1] = mpc.utils.odomY
                    mpc.real_state = np.array([mpc.utils.gps_x, mpc.utils.gps_y, mpc.utils.yaw])
                else:
                    mpc.current_state[0] = mpc.utils.gps_x
                    mpc.current_state[1] = mpc.utils.gps_y
            mpc.x_errors.append(mpc.utils.gps_x - mpc.next_trajectories[0, 0])
            mpc.y_errors.append(mpc.utils.gps_y - mpc.next_trajectories[0, 1])
        else:
            mpc.x_errors.append(mpc.current_state[0] - mpc.next_trajectories[0, 0])
            mpc.y_errors.append(mpc.current_state[1] - mpc.next_trajectories[0, 1])
        mpc.x_refs.append(mpc.next_trajectories[0, :])
        mpc.yaw_errors.append(mpc.current_state[2] - mpc.next_trajectories[0, 2])
        # initial_state_with_latency = mpc.predict_state_with_latency(mpc.current_state)
        initial_state_with_latency = mpc.current_state
        mpc.opti.set_value(mpc.opt_x_ref[0, :], initial_state_with_latency)
        # print("initial_state_with_latency: ", initial_state_with_latency, "current_state: ", mpc.current_state)
        target_waypoint_index = mpc.find_next_waypoint(initial_state_with_latency[0], initial_state_with_latency[1])
        # make sure yaw between -pi and pi
        initial_state_with_latency[2] = np.arctan2(np.sin(initial_state_with_latency[2]), np.cos(initial_state_with_latency[2]))
        mpc.current_state[2] = np.arctan2(np.sin(mpc.current_state[2]), np.cos(mpc.current_state[2]))
        mpc.next_trajectories, mpc.next_controls = mpc.desired_command_and_trajectory(target_waypoint_index)
        t_ = time.time()
        u_res, x_m = mpc.update_and_solve()
        t2 = time.time()- t_
        mpc.index_t.append(t2)
        # if target_waypoint_index < length-1:
            # print("i) ", mpc.mpciter,"ref_traj:", np.around(mpc.next_trajectories, decimals=2)[0], 
            #     "cur:", np.around(initial_state_with_latency, decimals=2), "ctrl:", 
            #     np.around(u_res[0, :], decimals=2), "idx:", target_waypoint_index, "time:", round(t2, 4),
            #     "kappa:", round(mpc.kappa[target_waypoint_index],2))
            # print("time", round(time.time()-t, 4))
        mpc.u_c.append(u_res[0, :])
        mpc.u_cc.append(u_res)
        mpc.t_c.append(mpc.t0)
        mpc.x_c.append(x_m)
        mpc.t0, mpc.current_state, mpc.u0, _ = mpc.shift_movement(mpc.t0, mpc.current_state, mpc.u_cc[-mpc.latency_num-1], x_m, mpc.f_np)
        # mpc.current_state = mpc.add_gaussian_noise(mpc.current_state)
        mpc.xx.append(mpc.real_state) if args.odom else mpc.xx.append(mpc.current_state)
        mpc.mpciter = mpc.mpciter + 1
        if mpc.gazebo:
            mpc.utils.steer_command = -float(u_res[0, 1]*180/np.pi)
            mpc.utils.velocity_command = float(u_res[0, 0])
            mpc.utils.publish_cmd_vel(steering_angle=mpc.utils.steer_command, velocity=mpc.utils.velocity_command)
            rate.sleep()
        if mpc.gazebo:
            print("i) ", mpc.mpciter,"ref_traj:", np.around(mpc.next_trajectories, decimals=2)[0], 
                "cur:", np.around(initial_state_with_latency, decimals=2), "ctrl:", 
                np.around(u_res[0, :], decimals=2), "idx:", target_waypoint_index, "time:", round(t2, 4),
                "kappa:", round(mpc.kappa[target_waypoint_index],2))
            # print("i) ", mpc.mpciter, "x: ", np.around(mpc.current_state[0], decimals=2), ", y: ",
            #     np.around(mpc.current_state[1], decimals=2), ", yaw: ",np.around(mpc.current_state[2], decimals=2),
            #         "u: ", np.around(u_res[0, :], decimals=2), "time: ", time.time()-t, "index: ", 
            #         target_waypoint_index, "kappa:", mpc.kappa[target_waypoint_index])
        # print("index: ", target_waypoint_index)
    print("done")
    if mpc.gazebo:
        try:
            call_trigger_service()
        except:
            print("service not available")
    ## after loop
    print(mpc.mpciter)
    t_v = np.array(mpc.index_t)
    print("mean solve time: ",t_v.mean(), "max: ", t_v.max(), "min: ", t_v.min(), "std: ", t_v.std(), "median: ", np.median(t_v))
    print((time.time() - mpc.start_time)/(mpc.mpciter))
    # print the control input, keep 2 digits after the decimal point
    mpc.u_c = np.array(mpc.u_c)
    mpc.x_c = np.array(mpc.x_c)

    print("average kappa: ", np.mean(mpc.kappa))
    average_speed = np.mean(mpc.u_c[:, 0])
    average_steer = np.mean(mpc.u_c[:, 1])
    
    delta_u_c = np.diff(mpc.u_c, axis=0)
    average_delta_speed = np.mean(np.abs(delta_u_c[:, 0]))
    average_delta_steer = np.mean(np.abs(delta_u_c[:, 1]))
    
    print(f"Average speed: {average_speed:.4f} m/s")
    print(f"Average steer angle: {average_steer:.4f} rad")
    print(f"Average change in speed: {average_delta_speed:.4f} m/s²")
    print(f"Average change in steer angle: {average_delta_steer:.4f} rad/s")

    average_x_error = np.mean(np.abs(mpc.x_errors))
    average_y_error = np.mean(np.abs(mpc.y_errors))
    mpc.yaw_errors = np.array(mpc.yaw_errors)
    mpc.yaw_errors = np.arctan2(np.sin(mpc.yaw_errors), np.cos(mpc.yaw_errors))
    average_yaw_error = np.mean(np.abs(mpc.yaw_errors))

    print(f"Average x error: {average_x_error:.4f} m")
    print(f"Average y error: {average_y_error:.4f} m")
    print(f"Average yaw error: {average_yaw_error:.4f} rad")

    stats = [average_speed, average_steer, average_delta_speed, average_delta_steer, average_x_error, average_y_error, average_yaw_error]

    ## draw function
    draw_result = Draw_MPC_tracking(obstacle = mpc.obstacle, rob_diam=0.3, init_state=mpc.init_state, 
                                    robot_states=mpc.xx, ref_states = mpc.x_refs, export_fig=mpc.export_fig, waypoints_x=mpc.waypoints_x, 
                                    waypoints_y=mpc.waypoints_y, stats = stats, costs = mpc.costs)
    