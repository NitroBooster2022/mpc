#!/usr/bin/env python3
from map import Map, Obstacle
import numpy as np
from reference_path import ReferencePath
from spatial_bicycle_models import BicycleModel
import matplotlib.pyplot as plt
from MPC import MPC
from scipy import sparse
import time
from matplotlib.animation import FuncAnimation

def main():
    # Select Simulation Mode | 'Sim_Track' or 'Real_Track'
    # sim_mode = 'Sim_Track'
    sim_mode = 'Real_Track'
    sim_mode = 'Sim_Track'

    # Simulation Environment. Mini-Car on track specifically designed to show-
    # case time-optimal driving.
    if sim_mode == 'Sim_Track':

        # Load map file
        map = Map(file_path='maps/white_map.png', origin=[-1, -2],
                  resolution=0.02)
        # map = Map(file_path='maps/modified_image.png', origin=[-1, -2], resolution=0.005)

        # Specify waypoints
        wp_x = [-0.75, -0.25, -0.25, 0.25, 0.25, 1.25, 1.25, 0.75, 0.75, 1.25,
                1.25, -0.75, -0.75, -0.25]
        wp_y = [-1.5, -1.5, -0.5, -0.5, -1.5, -1.5, -1, -1, -0.5, -0.5, 0, 0,
                -1.5, -1.5]
        wp_x = [3.61, 3.98, 4.34, 5.09, 5.27, 5.27, 5.27, 5.27, 5.27, 5.41, 5.71, 6.09, 6.47, 6.84, 7.22, 7.6, 7.98, 8.35, 8.73, 9.11, 9.39, 9.74, 9.68, 9.65, 9.44, 9.32, 9.31, 9.3, 9.31, 9.32, 9.34, 9.38, 9.44, 9.53, 9.62, 9.76, 9.9, 9.98, 10.0, 9.98, 9.9, 9.75, 9.55, 9.3, 9.02, 8.7, 8.34, 7.97, 7.59, 7.21, 6.83, 6.44, 6.06, 5.68, 5.26, 5.27, 5.27, 5.27, 5.27, 5.09, 4.35, 3.96, 3.63, 2.86, 3.05, 3.05, 3.04, 3.04, 3.04, 3.03, 3.04, 2.86, 3.61]
        # print("wp_x: ", len(wp_x))
        wp_y = [8.02, 8.02, 8.01, 8.19, 8.94, 9.32, 9.7, 10.08, 10.46, 10.82, 11.05, 11.07, 11.07, 11.07, 11.07, 11.07, 11.07, 11.07, 11.06, 11.06, 10.81, 10.65, 10.27, 9.9, 9.57, 9.22, 8.84, 8.46, 8.09, 7.71, 7.33, 6.95, 6.58, 6.22, 5.85, 5.5, 5.15, 4.78, 4.4, 4.02, 3.65, 3.3, 2.98, 2.69, 2.44, 2.24, 2.12, 2.02, 1.99, 2.0, 2.02, 2.03, 2.01, 2.03, 2.2, 2.58, 2.96, 3.34, 3.61, 4.36, 4.54, 4.54, 4.53, 4.35, 5.11, 5.49, 5.87, 6.25, 6.64, 7.02, 7.46, 8.21, 8.02]
        wp1 = np.load("waypoints/speedrun1_noint.npy")
        wp2 = np.load("waypoints/speedrun2_noint.npy")
        wp3 = np.load("waypoints/speedrun3_noint.npy")
        wp4 = np.load("waypoints/speedrun4_noint.npy")
        wp = np.concatenate((wp1, wp2, wp3, wp4), axis=1)
        # wp = wp1
        print("wp: ", wp.shape, wp)
        wp_x = wp[0]
        print("wp_x: ", wp_x.shape)
        wp_y = wp[1]
        # Specify path resolution
        path_resolution = 0.05  # m / wp

        # Create smoothed reference path
        reference_path = ReferencePath(map, wp_x, wp_y, path_resolution,
                                       smoothing_distance=5, max_width=0.23,
                                       circular=True)

        # Add obstacles
        use_obstacles = False
        if use_obstacles:
            obs1 = Obstacle(cx=0.0, cy=0.0, radius=0.05)
            obs2 = Obstacle(cx=-0.8, cy=-0.5, radius=0.08)
            obs3 = Obstacle(cx=-0.7, cy=-1.5, radius=0.05)
            obs4 = Obstacle(cx=-0.3, cy=-1.0, radius=0.08)
            obs5 = Obstacle(cx=0.27, cy=-1.0, radius=0.05)
            obs6 = Obstacle(cx=0.78, cy=-1.47, radius=0.05)
            obs7 = Obstacle(cx=0.73, cy=-0.9, radius=0.07)
            obs8 = Obstacle(cx=1.2, cy=0.0, radius=0.08)
            obs9 = Obstacle(cx=0.67, cy=-0.05, radius=0.06)
            map.add_obstacles([obs1, obs2, obs3, obs4, obs5, obs6, obs7,
                                          obs8, obs9])

        # Instantiate motion model
        car = BicycleModel(length=0.32, width=0.15,
                           reference_path=reference_path, Ts=0.05)

    # Real-World Environment. Track used for testing the algorithm on a 1:12
    # RC car.
    elif sim_mode == 'Real_Track':

        # Load map file
        map = Map(file_path='maps/real_map.png', origin=(-30.0, -24.0),
                  resolution=0.06)

        # Specify waypoints
        wp_x = [-9.169, 11.9, 7.3, -6.95]
        wp_y = [-15.678, 10.9, 14.5, -3.31]

        # Specify path resolution
        path_resolution = 0.20  # m / wp

        # Create smoothed reference path
        reference_path = ReferencePath(map, wp_x, wp_y, path_resolution,
                                       smoothing_distance=5, max_width=1.50,
                                       circular=False)

        # Add obstacles
        add_obstacles = False
        if add_obstacles:
            obs1 = Obstacle(cx=-6.3, cy=-11.1, radius=0.20)
            obs2 = Obstacle(cx=-2.2, cy=-6.8, radius=0.25)
            obs4 = Obstacle(cx=2.0, cy=-0.2, radius=0.25)
            obs8 = Obstacle(cx=6.0, cy=5.0, radius=0.3)
            obs9 = Obstacle(cx=7.42, cy=4.97, radius=0.3)
            map.add_obstacles([obs1, obs2, obs4, obs8, obs9])

        # Instantiate motion model
        car = BicycleModel(length=0.30, width=0.20,
                           reference_path=reference_path, Ts=0.05)

    else:
        print('Invalid Simulation Mode!')
        map, wp_x, wp_y, path_resolution, reference_path, car \
            = None, None, None, None, None, None
        exit(1)

    ##############
    # Controller #
    ##############

    N = 10 # Prediction horizon
    Q = sparse.diags([1.0, 0.0, 0.0]) # Q represents the cost of the state
    R = sparse.diags([0.5, 0.0]) # R represents the cost of the input
    QN = sparse.diags([1.0, 0.0, 0.0]) # QN represents the cost of the final state

    v_max = 1.0  # m/s
    delta_max = 0.4  # rad
    ay_max = 4.0  # m/s^2
    InputConstraints = {'umin': np.array([0.0, -np.tan(delta_max)/car.length]),
                        'umax': np.array([v_max, np.tan(delta_max)/car.length])}
    print("InputConstraints: ", InputConstraints)
    # InputConstraints = {'umin': np.array([0.0, -0.45]),
    #                     'umax': np.array([v_max, 0.45])}
    StateConstraints = {'xmin': np.array([-np.inf, -np.inf, -np.inf]),
                        'xmax': np.array([np.inf, np.inf, np.inf])}
    mpc = MPC(car, N, Q, R, QN, StateConstraints, InputConstraints, ay_max)

    # Compute speed profile
    a_min = -0.1  # m/s^2
    a_max = 0.5  # m/s^2
    SpeedProfileConstraints = {'a_min': a_min, 'a_max': a_max,
                               'v_min': 0.0, 'v_max': v_max, 'ay_max': ay_max}
    car.reference_path.compute_speed_profile(SpeedProfileConstraints)

    ##############
    # Simulation #
    ##############

    # Set simulation time to zero
    t = 0.0

    # Logging containers
    x_log = [car.temporal_state.x]
    y_log = [car.temporal_state.y]
    v_log = [0.0]

    t1 = time.time()
    times = []
    fig = plt.figure()

    def update(frame):
        nonlocal t, t1

        time1 = time.time() - t1
        t1 = time.time()
        times.append(time1)

        # Get control signals
        u = mpc.get_control()

        # Simulate car
        car.drive(u)

        # Log car state
        x_log.append(car.temporal_state.x)
        y_log.append(car.temporal_state.y)
        v_log.append(u[0])

        # Increment simulation time
        t += car.Ts

        # Plot path and drivable area
        reference_path.show()

        # Plot car
        car.show()

        # Plot MPC prediction
        mpc.show_prediction()
        # Set figure title
        plt.title('MPC Simulation: v(t): {:.2f}, delta(t): {:.2f}, Duration: {:.2f} s'.format(u[0], u[1], t))
        plt.axis('off')
        plt.pause(0.0001)

    # Calculate the number of frames required for the animation
    num_frames = int(reference_path.length / car.Ts)-1  # This is just an approximation. Adjust as needed.

    ani = FuncAnimation(fig, update, frames=range(num_frames), repeat=False)

    # Save the animation as a GIF
    ani.save('gifs/simulation2.gif', writer='pillow', fps=100)
if __name__ == '__main__':
    main()
    
