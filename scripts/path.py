#!/usr/bin/env python3

import time
import numpy as np
import os
from scipy.interpolate import UnivariateSpline, splprep, splev
from global_planner import GlobalPlanner

def smooth_yaw_angles(yaw_angles):
    # Calculate the differences between adjacent angles
    diffs = np.diff(yaw_angles)
    # Find where the difference is greater than pi and adjust
    diffs[diffs > np.pi] -= 2 * np.pi
    diffs[diffs < -np.pi] += 2 * np.pi
    # Compute the smoothed yaw angles
    smooth_yaw = np.concatenate(([yaw_angles[0]], yaw_angles[0] + np.cumsum(diffs)))
    return smooth_yaw

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
            # print("filtered out waypoint: ",i, waypoints[i])
            continue
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

class Path:
    def __init__(self, v_ref, N, T, x0=None):
        self.v_ref = v_ref
        self.N = N
        self.region_of_acceptance = 0.05
        self.global_planner = GlobalPlanner()
        run1, _ = self.global_planner.plan_path(86, 467)
        run2, _ = self.global_planner.plan_path(467, 302)
        run3, _ = self.global_planner.plan_path(302, 264)
        run4, _ = self.global_planner.plan_path(264, 81)
        runs = [run1, run2, run3, run4]
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
        # append x0 to run1
        if x0 is not None:
            run1 = np.hstack((x0[0:2].reshape(2,1), run1))
        runs = [run1, run2, run3, run4, run5]

        # Compute path lengths 
        path_lengths = [np.sum(np.linalg.norm(run[:, 1:] - run[:, :-1], axis=0)) for run in runs]
        self.density = 1/self.v_ref/T # wp/m
        print("density: ", self.density)
        for i, length in enumerate(path_lengths):
            print(i, ") path length: ", length)
            runs[i] = interpolate_waypoints(runs[i].T, int(np.ceil(length*self.density)))
        # Combine all runs into a single set of waypoints
        self.waypoints = np.vstack(runs)
        print("waypoints: ", self.waypoints.shape)
        self.waypoints = filter_waypoints(self.waypoints, 0.01).T
        # Calculate the total path length of the waypoints
        total_path_length = np.sum(np.linalg.norm(self.waypoints[:, 1:] - self.waypoints[:, :-1], axis=0))
        print("total path length: ", total_path_length)
        # self.waypoints = np.vstack((self.waypoints[1], self.waypoints[0])) #flip x and y
        # self.waypoints[0] = 15-self.waypoints[0] # flip x
        # self.waypoints[1] = 15-self.waypoints[1] # flip y
        self.waypoints_x = self.waypoints[0, :]
        self.waypoints_y = self.waypoints[1, :]
        # filepath = os.path.dirname(os.path.abspath(__file__))
        # wpts = np.load(os.path.join(filepath, 'waypoints/parallel_park.npy'))
        # self.waypoints_x = wpts[:, 0]+5
        # self.waypoints_y = wpts[:, 1]+5
        start_index = int(len(self.waypoints_x)*0)
        end_index = int(len(self.waypoints_x)*1)
        theta_ref = np.arctan2(self.waypoints_y[start_index+1] - self.waypoints_y[start_index],
                                self.waypoints_x[start_index+1] - self.waypoints_x[start_index])        
        self.init_state = np.array([self.waypoints_x[start_index], self.waypoints_y[start_index], theta_ref])
        print("init_state: ", self.init_state)

        self.num_waypoints = len(self.waypoints_x)
        print("num_waypoints: ", self.num_waypoints)
        self.kappa, self.wp_theta, self.wp_normals = compute_smooth_curvature(self.waypoints_x, self.waypoints_y)
        # linear speed profile
        self.v_refs  = self.v_ref / (1 + np.abs(self.kappa))*(1 + np.abs(self.kappa))
        #nonlinear speed profile
        # K=1
        # epsilon=1
        # self.v_refs = self.v_ref * (K/(abs(self.kappa) + epsilon))**0.5
        num_ramp_waypoints = int(1 * self.density) # 1 meters
        num_ramp_waypoints = min(num_ramp_waypoints, len(self.v_refs))
        # Linearly increase the reference speed from 0 to v_ref over the first 2 meters
        self.v_refs[:num_ramp_waypoints] = np.linspace(0, self.v_ref, num_ramp_waypoints)
        self.v_refs[-2:] = 0 # stop at the end
        # print("v_ref: ", self.v_ref)
        # print("vrefs: ", self.v_refs[0:100])
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
        self.state_refs[:,2] = smooth_yaw_angles(self.state_refs[:,2])
        # for i in range(len(self.state_refs)):
        #     print(i, self.state_refs[i,2])
        # exit()
        print("state_refs: ", self.state_refs.shape, ", input_refs: ", self.input_refs.shape)

    def change_lane(self, start_index, end_index, normals, shift_distance=0.36-0.1):
        self.state_refs[start_index:end_index,0] += normals[start_index:end_index, 0] * shift_distance
        self.state_refs[start_index:end_index,1] += normals[start_index:end_index, 1] * shift_distance
        return self.state_refs

    def desired_command_and_trajectory(self, index):
        return self.state_refs[index:index + self.N + 1], self.input_refs[index:index + self.N]