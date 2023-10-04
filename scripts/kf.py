#! /usr/bin/env python3

import numpy as np
import math
from sensor_msgs.msg import Imu
from utils.msg import encoder, Car_command
import rospy
from std_msgs.msg import Float32, Header, String
from gazebo_msgs.msg import ModelStates
import tf

def shutdown():
    pub = rospy.Publisher("/automobile/command", String, queue_size=3)
    msg = String()
    msg2 = String()
    # msg.data = '{"action":"3","brake (steerAngle)":'+str(0.0)+'}'
    msg.data = '{"action":"1","speed":'+str(0.0)+'}'
    msg2.data = '{"action":"2","steerAngle":'+str(0.0)+'}'
    for haha in range(10):
        pub.publish(msg2)
        pub.publish(msg)
        rospy.sleep(0.1)
# def prediction_step(x, u, P, Q, dt, L):
#     # Unpack the state and control variables
#     x_pos, y_pos, yaw = x
#     v, steer = u

#     # Define the state transition matrix (F)
#     F = np.array([
#         [1, 0, -v*dt*np.sin(yaw)],
#         [0, 1, v*dt*np.cos(yaw)],
#         [0, 0, 1]
#     ])
    
#     # Define the control input matrix (B)
#     B = np.array([
#         [dt*np.cos(yaw), 0],
#         [dt*np.sin(yaw), 0],
#         [dt*np.tan(steer)/L, v*dt/(L*np.cos(steer)**2)]
#     ])
    
#     # Predict the next state
#     x_pred = np.dot(F, x) + np.dot(B, u)    
    
#     # Wrap the yaw angle between -pi and pi
#     x_pred[2] = np.fmod(x_pred[2] + np.pi, 2 * np.pi) - np.pi

#     # Predict the next covariance matrix
#     P = np.dot(np.dot(F, P), F.T) + Q

#     return x_pred, P
def prediction_step(x, u, P, Q, dt, L):
    # Unpack the state and control variables
    x_pos, y_pos, yaw = x
    v, steer = u

    # Define the state transition matrix (F)
    F = np.array([
        [1, 0, -v*dt*np.sin(yaw)],
        [0, 1, v*dt*np.cos(yaw)],
        [0, 0, 1]
    ])
    
    # Predict the next state
    x_pred = x + dt*np.array([v*np.cos(yaw), v*np.sin(yaw), v*np.tan(steer)/L])  
    
    # Wrap the yaw angle between -pi and pi
    x_pred[2] = np.fmod(x_pred[2] + np.pi, 2 * np.pi) - np.pi

    # Predict the next covariance matrix
    P = np.dot(np.dot(F, P), F.T) + Q

    return x_pred, P
def update_step(x_pred, P_pred, z, R, v_measured):
    # Unpack the state variables
    x, y, yaw = x_pred

    # Define the measurement function
    h_x = np.array([
        v_measured,  # v
        yaw,  # yaw
        v_measured * np.cos(yaw) - v_measured * np.sin(yaw),  # ax
        v_measured * np.sin(yaw) + v_measured * np.cos(yaw),  # ay
    ])

    # Define the Jacobian of the measurement function (H)
    H = np.array([
        [0, 0, 0],
        [0, 0, 1],
        [0, 0, -v_measured*np.sin(yaw) - v_measured*np.cos(yaw)],
        [0, 0, -v_measured*np.sin(yaw) + v_measured*np.cos(yaw)]
    ])

    # Compute the Kalman gain
    S = np.dot(np.dot(H, P_pred), H.T) + R
    K = np.dot(np.dot(P_pred, H.T), np.linalg.inv(S))

    # Compute the measurement residual
    y_residual = z - h_x

    # Update the state estimate
    x_est = x_pred + np.dot(K, y_residual)

    # Wrap the yaw angle between -pi and pi
    x_est[2] = np.fmod(x_est[2] + np.pi, 2 * np.pi) - np.pi

    # Update the covariance matrix
    I = np.eye(3)
    P_est = np.dot((I - np.dot(K, H)), P_pred)

    return x_est, P_est

class EKF_Node:
    def __init__(self):
        self.toggle = True
        self.x = np.zeros(3)  # Initial state [x, y, yaw]
        self.P = np.diag([0.01, 0.01, np.deg2rad(0.1)])**2  # Initial covariance matrix
        self.Q = np.diag([0.03, 0.03, np.deg2rad(0.3)])**2  # Assuming some nominal process noise values
        self.R = np.diag([0.01, np.deg2rad(0.3), 0.03, 0.03])**2
        self.L = 0.27        
        self.v_meas = 0
        self.u = np.array([0, 0])
        self.x_ground = np.zeros(3)
        self.v_ground = 0
        self.car_index = None
        self.initializationTimer = None

        print("init EKF node")
        rospy.init_node('EKF_Node', anonymous=True)
        self.rate = rospy.Rate(100)
        self.last_prediction_time = rospy.Time.now()
        self.last_update_time = rospy.Time.now()
        self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback, queue_size=3)
        self.encoder_sub = rospy.Subscriber("/encoder", encoder, self.encoder_callback, queue_size = 3)
        self.command_sub = rospy.Subscriber("/command", Car_command, self.command_callback, queue_size = 3)
        self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback, queue_size = 3)
        
        rospy.on_shutdown(shutdown)

    def imu_callback(self, msg):
        yaw = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])[2]
        self.x[2] = yaw
        # z = np.array([self.v_meas, yaw, msg.linear_acceleration.x, msg.linear_acceleration.y])
        # self.x, self.P = update_step(self.x, self.P, z, self.R, self.v_meas)

    def encoder_callback(self, msg):
        self.v_meas = msg.speed
        self.u[0] = msg.speed
        current_time = msg.header.stamp
        dt_prediction = (current_time - self.last_prediction_time).to_sec()
        self.last_prediction_time = current_time
        self.x, self.P = prediction_step(self.x, self.u, self.P, self.Q, dt_prediction, self.L)

        error = self.x_ground - self.x
        error[2] = np.fmod(error[2] + np.pi, 2 * np.pi) - np.pi
        print(np.around(dt_prediction,3),"x: ", self.x, "x_ground: ", self.x_ground, "error: ", error)
    def command_callback(self, msg):
        self.u = np.array([msg.speed, msg.steering_angle])
        

    def model_callback(self, model):
        if self.car_index is None:
            try:
                self.car_idx = model.name.index("automobile")
            except ValueError:
                return
        if self.initializationTimer is None:
            self.initializationTimer = rospy.Time.now() + rospy.Duration(3.57)
        elif rospy.Time.now() < self.initializationTimer:
            self.x[2] = self.x_ground[2]
            print(f"intializing... gps_x: {self.x_ground[0]:.2f}, gps_y: {self.x_ground[1]:.2f}")
            self.x[0] = self.x_ground[0]
            self.x[1] = self.x_ground[1]
            print(f"odomX: {self.x[0]:.2f}, odomY: {self.x[1]:.2f}")
            return
        self.car_pose = model.pose[self.car_idx]
        self.car_inertial = model.twist[self.car_idx]
        self.x_ground = np.array([self.car_pose.position.x, self.car_pose.position.y, self.car_pose.orientation.z])
        x_speed = self.car_inertial.linear.x
        y_speed = self.car_inertial.linear.y
        speedYaw = math.atan2(y_speed, x_speed)
        speed = math.sqrt(x_speed**2 + y_speed**2)
        angle_diff = (speedYaw - self.x[2] + math.pi) % (2 * math.pi) - math.pi
        if abs(angle_diff) > 3 * math.pi / 4: 
            speed *= -1
        self.v_ground = speed

if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            node = EKF_Node()
            node.rate.sleep()
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
