#! /usr/bin/env python3

import rospy
import numpy as np
from utils.msg import localisation, IMU, encoder
import math
from gazebo_msgs.msg import ModelStates
import message_filters
from message_filters import ApproximateTimeSynchronizer
from std_msgs.msg import Float32, Header
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf
from nav_msgs.msg import Odometry 
import time
from utility import Utility
from sensor_msgs.msg import Imu
from utils.msg import Lane

class Odom():
    def __init__(self, useIMU=False):
        rospy.init_node('control_node', anonymous=True)
        self.useIMU = useIMU
        self.utils = Utility(useIMU=useIMU, subLane=True, subImu=False, subModel=False)
        
        # Subscribe to topics
        if useIMU:
            self.imu_sub = message_filters.Subscriber("/automobile/IMU", IMU, queue_size=3)
        else:
            self.imu_sub = message_filters.Subscriber("/imu", Imu, queue_size=3)
        self.model_sub = message_filters.Subscriber("/gazebo/model_states", ModelStates, queue_size=3)
        # self.encoder_sub = message_filters.Subscriber("/automobile/encoder", encoder, queue_size=3)
        # self.localization_sub = message_filters.Subscriber("/automobile/localisation", localisation, queue_size=3)

        self.subscribers = []
        # self.subscribers.append(self.localization_sub)
        # self.subscribers.append(self.encoder_sub)
        self.subscribers.append(self.imu_sub)
        self.subscribers.append(self.model_sub)
        
        # Create an instance of TimeSynchronizer
        ts = ApproximateTimeSynchronizer(self.subscribers, queue_size=3, slop=0.0015, allow_headerless=True)
        ts.registerCallback(self.callback)

    def callback(self, imu, model):
        self.utils.imu = imu
        self.utils.model = model
        self.utils.lane_follow()
        self.utils.publish_odom()

if __name__ == '__main__':
    node = Odom()
    while not rospy.is_shutdown():
        rospy.spin()
        # node.rate.sleep()
        