#!/usr/bin/env python3
# coding=utf-8

import numpy as np
from matplotlib import pyplot as plt
import matplotlib.animation as animation
import matplotlib.patches as mpatches
from scipy.interpolate import interp1d
import cv2

class Draw_MPC_tracking(object):
    def __init__(self, robot_states: list, ref_states:list, init_state: np.array, 
                 obstacle: np.array, rob_diam=0.3,  export_fig=None
                 , xmin=-1.0, xmax=15, ymin=-1, ymax=15, waypoints_x = None, waypoints_y = None, 
                 spline_points_x = None, spline_points_y = None, stats = None, costs = None):
        self.init_state = init_state
        self.robot_states = robot_states
        self.rob_radius = rob_diam
        self.waypoints_x = waypoints_x
        self.waypoints_y = waypoints_y
        self.spline_points_x = spline_points_x
        self.spline_points_y = spline_points_y
        self.stats = stats
        self.costs = costs
        self.image = cv2.imread('/home/simonli/Documents/husky_ws/src/control/scripts/imgs/Competition_track_graph.png')
        self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        self.image = np.array(self.image).T
        # resize the image to 500 by 500
        self.image = cv2.resize(self.image, (100, 100), interpolation=cv2.INTER_AREA)

        if obstacle is not None:
            self.obstacle = obstacle
        else:
            print('no obstacle given, break')
        self.fig = plt.figure()
        # self.ax = plt.axes(xlim=(-1.0, 15), ylim=(-1, 15))
        self.ax = plt.axes(xlim=(xmin, xmax), ylim=(ymin, ymax))
        # self.fig.set_size_inches(7, 6.5)
        self.obstacle_circles = [] 

        self.background_image = plt.imread('/home/simonli/Documents/husky_ws/src/control/scripts/imgs/Competition_track_graph.png') 

        # init for plot
        self.animation_init()

        self.ani = animation.FuncAnimation(self.fig, self.animation_loop, range(len(self.robot_states)),
                                           init_func=self.animation_init, interval=100, repeat=False)
        self.ref_states = np.array(ref_states)  
        self.robot_states = np.array(robot_states)
        self.interpolate_trajectories()
        self.calculate_statistics()
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('MPC Tracking Performance')
        self.plot_and_save(output_path=export_fig.replace('gifs', 'plots')+ '.png')
        # self.ax.legend(loc='center', frameon=True, bbox_to_anchor=(0.8, 0.1))
        plt.grid('--')
        if export_fig is not None:
            self.ani.save(export_fig+'.gif', writer='imagemagick', fps=100)
        plt.show()


    def animation_init(self, ):
        if self.spline_points_x is not None:
            self.ax.plot(self.spline_points_x, self.spline_points_y, '-g', label='Continuous Path', linewidth=0.357)
       
        # if self.waypoints_x is not None:
        #     self.ax.plot(self.waypoints_x[:], self.waypoints_y[:], 'go', label='Waypoints',markersize=1)
            # draw waypoints smaller circle
        
        # self.ax.imshow(self.background_image, extent=[0, 15, 0, 15], aspect='auto')

        # draw the initial position of the robot
        self.init_robot_position = plt.Circle(self.init_state[:2], self.rob_radius, color='r', fill=False)
        self.ax.add_artist(self.init_robot_position)
        self.robot_body = plt.Circle(self.init_state[:2], self.rob_radius, color='r', fill=False)
        self.ax.add_artist(self.robot_body)
        self.robot_arr = mpatches.Arrow(self.init_state[0], self.init_state[1],
                                        self.rob_radius * np.cos(self.init_state[2]),
                                        self.rob_radius * np.sin(self.init_state[2]), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        for obs in self.obstacle:  # Iterate over each obstacle
            obs_circle = plt.Circle(obs[:2], obs[2], color='purple', fill=True)
            self.ax.add_artist(obs_circle)
            self.obstacle_circles.append(obs_circle)
        return
    
    def interpolate_trajectories(self):
        t = np.linspace(0, 1, len(self.ref_states))
        
        self.ref_traj_interp_x = interp1d(t, self.ref_states[:, 0], kind='cubic')
        self.ref_traj_interp_y = interp1d(t, self.ref_states[:, 1], kind='cubic')
        
        self.true_traj_interp_x = interp1d(t, self.robot_states[:, 0], kind='cubic')
        self.true_traj_interp_y = interp1d(t, self.robot_states[:, 1], kind='cubic')

    def calculate_statistics(self):
        self.mean_error_x = np.mean(np.abs(self.robot_states[:, 0] - self.ref_states[:, 0]))
        self.mean_error_y = np.mean(np.abs(self.robot_states[:, 1] - self.ref_states[:, 1]))
        self.max_error_x = np.max(np.abs(self.robot_states[:, 0] - self.ref_states[:, 0]))
        self.max_error_y = np.max(np.abs(self.robot_states[:, 1] - self.ref_states[:, 1]))
        self.min_error_x = np.min(np.abs(self.robot_states[:, 0] - self.ref_states[:, 0]))
        self.min_error_y = np.min(np.abs(self.robot_states[:, 1] - self.ref_states[:, 1]))

    def plot_and_save(self, output_path='output.png'):
        # Plot the interpolated trajectories
        t = np.linspace(0, 1, 1000)  # 1000 points for a smooth curve
        ref_x = self.ref_traj_interp_x(t)
        ref_y = self.ref_traj_interp_y(t)
        true_x = self.true_traj_interp_x(t)
        true_y = self.true_traj_interp_y(t)
        
        # Plot the trajectories
        self.ax.plot(ref_x, ref_y, label='Reference trajectory', linewidth=1.25, color='purple')
        self.ax.plot(true_x, true_y, label='Robot trajectory', linewidth=1.25, color='orange')
        
        self.ax.legend(loc='center', frameon=True, bbox_to_anchor=(0.83, 0.04))
        
        stats_text = f"""Statistics\nAverage Speed: {self.stats[0]:.3f}\nAverage Steer: {self.stats[1]:.3f}\nAverage Delta Speed: {self.stats[2]:.3f}\nAverage Delta Steer: {self.stats[3]:.3f}\nAverage X Error: {self.stats[4]:.3f}\nAverage Y Error: {self.stats[5]:.3f}\nAverage Yaw Error: {self.stats[6]:.3f}"""
        costs_text = f"""Cost Values\nXY Cost: {self.costs[0]:.2f}\nYaw Cost: {self.costs[1]:.2f}\nV Cost: {self.costs[2]:.2f}\nSteer Cost: {self.costs[3]:.2f}\nDelta V Cost: {self.costs[4]:.2f}\nDelta Steer Cost: {self.costs[5]:.2f}"""
        self.ax.text(0.123, 0.685, stats_text, transform=self.ax.transAxes, verticalalignment='center', fontsize=10, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        self.ax.text(0.123, 0.325, costs_text, transform=self.ax.transAxes, verticalalignment='center', fontsize=10, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        plt.savefig(output_path)

    def animation_loop(self, indx):
        position = self.robot_states[indx][:2]
        orientation = self.robot_states[indx][2]
        self.robot_body.center = position
        self.robot_arr.remove()
        self.robot_arr = mpatches.Arrow(position[0], position[1], self.rob_radius * np.cos(orientation),
                                        self.rob_radius * np.sin(orientation), width=0.2, color='r')
        self.ax.add_patch(self.robot_arr)
        return self.robot_arr, self.robot_body