#!/usr/bin/env python3
import rospy
import tf.transformations
from sensor_msgs.msg import Imu

def imu_callback(data):
    # Extract the quaternion
    orientation_q = data.orientation
    quaternion = (
        orientation_q.x,
        orientation_q.y,
        orientation_q.z,
        orientation_q.w
    )
    
    # Convert the quaternion to Euler angles
    euler = tf.transformations.euler_from_quaternion(quaternion)
    # euler is a tuple (roll, pitch, yaw)

    yaw = euler[2] # Extract the yaw (around the z-axis)
    rospy.loginfo("Yaw: {:.4f}".format(yaw))

def listener():
    rospy.init_node('imu_yaw_listener', anonymous=True)

    # Subscribe to the IMU topic
    rospy.Subscriber("camera/imu", Imu, imu_callback)

    # Keep the node running until shut down
    rospy.spin()

if __name__ == '__main__':
    listener()
