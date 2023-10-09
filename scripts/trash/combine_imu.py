#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu

class IMUCombiner:
    def __init__(self):
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)

        rospy.Subscriber('/camera/accel/sample', Imu, self.accel_callback)
        rospy.Subscriber('/camera/gyro/sample', Imu, self.gyro_callback)

        self.latest_accel = None
        self.latest_gyro = None

    def accel_callback(self, data):
        self.latest_accel = data
        self.publish_combined_imu()

    def gyro_callback(self, data):
        self.latest_gyro = data
        self.publish_combined_imu()

    def publish_combined_imu(self):
        if self.latest_accel is not None and self.latest_gyro is not None:
            combined_imu = Imu()
            combined_imu.header.stamp = rospy.Time.now()
            combined_imu.header.frame_id = "imu_frame"
            combined_imu.angular_velocity = self.latest_gyro.angular_velocity
            combined_imu.angular_velocity_covariance = self.latest_gyro.angular_velocity_covariance
            combined_imu.linear_acceleration = self.latest_accel.linear_acceleration
            combined_imu.linear_acceleration_covariance = self.latest_accel.linear_acceleration_covariance

            self.imu_pub.publish(combined_imu)

if __name__ == '__main__':
    rospy.init_node('imu_combiner', anonymous=False)
    combiner = IMUCombiner()
    rospy.spin()
