# mpc

## Model Predictive Control development for BFMC

It uses Gazebo model information and an EKF from the ROS package robot_localization and a predeterminate waypoint list as input and plans and simulates the input commands to complete the path as quickly and accurately as possible. (The simulation can be done in Gazebo or without it)

## Structure

- config: EKF parameters and how to clear them in simulation

- include: Packges and function required by MPC

- launch: Launch files for MPC

- scripts and src: Programs to run MPC and display simulation results.
