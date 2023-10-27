#!/usr/bin/env python3
import time
from draw import Draw_MPC_tracking
import casadi as ca
import numpy as np
import os
from path import Path

class MPC:
    def __init__(self, gazebo):
        self.gazebo = gazebo
        gaz_bool = "" if not gazebo else "_gazebo_"
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
        
        self.path = Path(self.v_ref, self.N)
        self.waypoints_x = self.path.waypoints_x
        self.waypoints_y = self.path.waypoints_y
        self.num_waypoints = self.path.num_waypoints
        self.wp_normals = self.path.wp_normals
        self.kappa = self.path.kappa
        self.density = self.path.density
        self.state_refs = self.path.state_refs
        self.input_refs = self.path.input_refs
        self.init_state = self.path.init_state

        filepath = os.path.dirname(os.path.abspath(__file__))
        name = 'speedrun'
        self.last_waypoint_index = 1
        
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
            self.opti.subject_to(self.opt_states[i+1, :]==x_next)
        # RK4 integration
        # for i in range(self.N):
        #     xi = self.opt_states[i, :]
        #     ui = self.opt_controls[i, :]
        #     k1 = self.f(xi, ui).T
        #     k2 = self.f(xi + 0.5*self.T*k1, ui).T
        #     k3 = self.f(xi + 0.5*self.T*k2, ui).T
        #     k4 = self.f(xi + self.T*k3, ui).T
        #     x_next = xi + (self.T/6.0)*(k1 + 2*k2 + 2*k3 + k4)
        #     self.opti.subject_to(self.opt_states[i+1, :] == x_next)


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
        self.opti.subject_to(self.opti.bounded(-np.pi*2, self.theta, np.pi*2))
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
        self.u_c = []
        for i in range(self.latency_num):
            self.u_c.append(np.zeros(2))
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
        state_noise = np.random.normal(0, state_noise_std_dev, self.current_state.shape)
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

    def update_and_solve(self):
        # set current reference state to the current state
        self.opti.set_value(self.opt_x_ref[0, :], self.current_state)
        self.target_waypoint_index = self.find_next_waypoint(self.current_state[0], self.current_state[1])
        # make sure yaw between -pi and pi
        self.current_state[2] = np.arctan2(np.sin(self.current_state[2]), np.cos(self.current_state[2]))
        self.next_trajectories, self.next_controls = self.path.desired_command_and_trajectory(self.target_waypoint_index)
        ## set parameter, here only update initial state of x (x0)
        self.opti.set_value(self.opt_x_ref[1:,:], self.next_trajectories[1:])
        self.opti.set_value(self.opt_u_ref, self.next_controls)
        ## provide the initial guess of the optimization targets
        self.opti.set_initial(self.opt_controls, self.u0.reshape(self.N, 2))
        self.opti.set_initial(self.opt_states, self.next_states)
        ## solve the problem once again
        self.sol = self.opti.solve()
        return self.sol.value(self.opt_controls)
    def get_predicted_states(self):
        return self.sol.value(self.opt_states)
    def integrate_next_states(self, u_res=None):
        x_m = self.get_predicted_states()
        self.u_c.append(u_res[0, :])
        self.t_c.append(self.t0)
        self.t0, self.current_state, self.u0, _ = self.shift_movement(self.t0, self.current_state, u_res, x_m, self.f_np)
        
    def update_current_state(self, x, y, yaw):
        self.current_state = np.array([x, y, yaw])
    def update_real_state(self, x, y, yaw):
        self.real_state = np.array([x, y, yaw])

    def compute_stats(self):
        ## after loop
        print("iter: ", self.mpciter)
        t_v = np.array(self.index_t)
        print("mean solve time: ",t_v.mean(), "max: ", t_v.max(), "min: ", t_v.min(), "std: ", t_v.std(), "median: ", np.median(t_v))
        print((time.time() - self.start_time)/(self.mpciter))
        # print the control input, keep 2 digits after the decimal point
        self.u_c = np.array(self.u_c)

        print("average kappa: ", np.mean(self.kappa))
        average_speed = np.mean(self.u_c[:, 0])
        average_steer = np.mean(self.u_c[:, 1])
        
        delta_u_c = np.diff(self.u_c, axis=0)
        average_delta_speed = np.mean(np.abs(delta_u_c[:, 0]))
        average_delta_steer = np.mean(np.abs(delta_u_c[:, 1]))
        
        print(f"Average speed: {average_speed:.4f} m/s")
        print(f"Average steer angle: {average_steer:.4f} rad")
        print(f"Average change in speed: {average_delta_speed:.4f} m/s²")
        print(f"Average change in steer angle: {average_delta_steer:.4f} rad/s")

        average_x_error = np.mean(np.abs(self.x_errors))
        average_y_error = np.mean(np.abs(self.y_errors))
        self.yaw_errors = np.array(self.yaw_errors)
        self.yaw_errors = np.arctan2(np.sin(self.yaw_errors), np.cos(self.yaw_errors))
        average_yaw_error = np.mean(np.abs(self.yaw_errors))

        print(f"Average x error: {average_x_error:.4f} m")
        print(f"Average y error: {average_y_error:.4f} m")
        print(f"Average yaw error: {average_yaw_error:.4f} rad")

        stats = [average_speed, average_steer, average_delta_speed, average_delta_steer, average_x_error, average_y_error, average_yaw_error]
        return stats
    
    def draw_result(self, stats):
        draw_result = Draw_MPC_tracking(self.u_c, obstacle = self.obstacle, rob_diam=0.3, init_state=self.init_state, 
                                    robot_states=self.xx, ref_states = self.x_refs, export_fig=self.export_fig, waypoints_x=self.waypoints_x, 
                                    waypoints_y=self.waypoints_y, stats = stats, costs = self.costs, xmin=self.x_min, xmax=self.x_max, ymin=self.y_min, ymax=self.y_max)
    
