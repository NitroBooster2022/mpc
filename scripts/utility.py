import rospy
import tf
from geometry_msgs.msg import TransformStamped
import math
from geometry_msgs.msg import TransformStamped
import time
import numpy as np
from nav_msgs.msg import Odometry
import tf2_ros
from std_msgs.msg import Header, String, Float32
from robot_localization.srv import SetPose
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates
from utils.msg import IMU, Lane
from sensor_msgs.msg import Imu

class Utility:
    def __init__(self, useIMU=True, subLane=True, subModel=False, subImu=False, pubOdom=False, useEkf=False):
        rospy.on_shutdown(self.stop_car)
        self.rateVal = 50.0
        self.rate = rospy.Rate(self.rateVal)
        self.pubOdom = pubOdom # publish odom in imu callback if true
        self.useEkf = useEkf
        self.useIMU = useIMU
        self.process_yaw = self.process_IMU if useIMU else self.process_Imu

        # Parameters
        self.wheelbase = 0.27
        self.odomRatio = 1
        self.maxspeed = 1.0
        self.center = -1
        self.image_center = 320
        self.p = 0.005
        self.d = 0.0005
        self.last = 0

        # static transforms
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.publish_static_transforms()
        self.br = tf2_ros.TransformBroadcaster()
        self.transform_stamped = TransformStamped()

        # Variables
        self.yaw = 1.5707
        self.velocity = 0.0
        self.odomX = 0
        self.odomY = 0
        self.odomYaw = 0
        self.gps_x = 0.0
        self.gps_y = 0.0
        self.steer = 0.0
        self.i = None
        
        #timers
        self.timerodom = rospy.Time.now()
        self.initializationTimer = None
        self.t1 = None
        self.timerpid = None

        # Flags
        self.initializationFlag = False

        self.pose_to_set = PoseWithCovarianceStamped()
        self.pose_to_set.header = Header()
        self.pose_to_set.header.frame_id = "chassis"

        covariance_value = 0.05
        # covariance_matrix = np.diag([covariance_value]*6).flatten().tolist()
        covariance_matrix = [covariance_value] * 6 + [0] * 30
        # Publishers
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=3)
        self.odom_msg = Odometry()
        self.odom_msg.pose.covariance = covariance_matrix
        self.odom_msg.twist.covariance = covariance_matrix

        self.odom1_pub = rospy.Publisher("odom1", Odometry, queue_size=3)
        self.odom1_msg = Odometry()
        covariance_matrix = [0.05] * 6 + [0] * 30
        self.odom1_msg.pose.covariance = covariance_matrix
        self.odom1_msg.twist.covariance = covariance_matrix

        self.cmd_vel_pub = rospy.Publisher("/automobile/command", String, queue_size=3)
        self.msg = String()
        self.msg2 = String()

        print("waiting for IMU message")
        if self.useIMU:
            rospy.wait_for_message("/automobile/IMU", IMU)
        else:
            rospy.wait_for_message("/imu", Imu)
        print("waiting for model_states message")
        rospy.wait_for_message("/gazebo/model_states", ModelStates)
        print("received message from IMU and model_states")
        
        # Subscribers
        self.model = ModelStates()
        self.imu = IMU()
        if subModel:
            self.model_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback, queue_size=3)
        self.useIMU = useIMU
        if subImu:
            if useIMU:
                self.imu_sub = rospy.Subscriber("/automobile/IMU", IMU, self.imu_callback, queue_size=3)
            else:
                self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback, queue_size=3)
        if subLane:
            self.lane_sub = rospy.Subscriber("/lane", Lane, self.lane_callback, queue_size=3)
            self.lane = Lane()
            print("waiting for lane message")
            rospy.wait_for_message("/lane", Lane)
            print("received message from lane")
            self.timerpid = rospy.Time.now()
            self.last = 0

    # Callbacks
    def lane_callback(self, lane):
        self.center = lane.center
    def imu_callback(self, imu):
        self.imu = imu
        if self.pubOdom:
            self.publish_odom()
    def model_callback(self, model):
        self.model = model
    def stop_car(self):
        pub = rospy.Publisher("/automobile/command", String, queue_size=3)
        msg = String()
        msg2 = String()
        msg.data = '{"action":"1","speed":'+str(0.0)+'}'
        msg2.data = '{"action":"2","steerAngle":'+str(0.0)+'}'
        for hsy in range(10):
            pub.publish(msg2)
            pub.publish(msg)
            time.sleep(0.1)
    def set_pose_using_service(self, x, y, yaw):
        self.pose_to_set.pose.pose.position.x = x
        self.pose_to_set.pose.pose.position.y = y
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        self.pose_to_set.pose.pose.orientation.x = quaternion[0]
        self.pose_to_set.pose.pose.orientation.y = quaternion[1]
        self.pose_to_set.pose.pose.orientation.z = quaternion[2]
        self.pose_to_set.pose.pose.orientation.w = quaternion[3]
        if self.useEkf:
            print("waiting for set_pose service")
            rospy.wait_for_service('/set_pose')  # Replace with the actual service name
        try:
            set_pose_service = rospy.ServiceProxy('/set_pose', SetPose)
            set_pose_service(self.pose_to_set)
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False
    def process_IMU(self, imu):
        self.yaw = imu.yaw
    def process_Imu(self, imu):
        self.yaw = tf.transformations.euler_from_quaternion([imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w])[2]
    def publish_odom(self):
        try:
            dt = time.time()-self.t1
        except:
            pass
        self.t1 = time.time()

        if self.i is None:
            try:
                self.i = self.model.name.index("automobile") # index of the car
            except ValueError:
                pass
        # set yaw
        self.process_yaw(self.imu)
        self.yaw = np.fmod(self.yaw, 2*math.pi)
        # set velocity
        self.car_inertial = self.model.twist[self.i]
        x_speed = self.car_inertial.linear.x
        y_speed = self.car_inertial.linear.y
        speedYaw = math.atan2(y_speed, x_speed)
        speed = math.sqrt(x_speed**2 + y_speed**2)
        angle_diff = (speedYaw - self.yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(angle_diff) > 3 * math.pi / 4: 
            speed *= -1
        self.velocity = speed
        # set gps
        self.gps_x = self.model.pose[self.i].position.x
        self.gps_y = 15+self.model.pose[self.i].position.y
        # initialize
        if not self.initializationFlag:
            self.initializationFlag = True
            print(f"intializing... gps_x: {self.gps_x:.2f}, gps_y: {self.gps_y:.2f}")
            self.setIntialPose(self.gps_x, self.gps_y, self.yaw)
            print(f"odomX: {self.odomX:.2f}, odomY: {self.odomY:.2f}")
            self.set_pose_using_service(self.odomX, self.odomY, self.yaw)
            return
        dx, dy, dyaw = self.update_states_rk4(self.velocity, self.steer)
        self.odomX += dx
        self.odomY += dy

        # print(f"dt: {dt:.2f}, odomx: {self.odomX:.2f}, odomy: {self.odomY:.2f}, gps_x: {self.gps_x:.2f}, gps_y: {self.gps_y:.2f}, x_error: {abs(self.odomX-self.gps_x):.2f}, y_error: {abs(self.odomY-self.gps_y):.2f}, yaw: {self.yaw:.2f}, steer: {self.steer:.2f}, speed: {self.velocity:.2f}")
       
        # Publish odom
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "chassis"
        self.odom_msg.pose.pose.position.x = self.odomX
        self.odom_msg.pose.pose.position.y = self.odomY
        self.odom_msg.pose.pose.position.z = 0.032939
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.yaw)
        self.odom_msg.pose.pose.orientation.x = quaternion[0]
        self.odom_msg.pose.pose.orientation.y = quaternion[1]
        self.odom_msg.pose.pose.orientation.z = quaternion[2]
        self.odom_msg.pose.pose.orientation.w = quaternion[3]
        self.odom_msg.twist.twist.linear.x = self.velocity*math.cos(self.yaw)
        self.odom_msg.twist.twist.linear.y = self.velocity*math.sin(self.yaw)
        self.odom_pub.publish(self.odom_msg)
        return

    def setIntialPose(self, x, y, yaw):
        self.odomX = x
        self.odomY = y
        self.odomYaw = yaw
    def lane_follow(self):
        self.steer = self.get_steering_angle()
        self.publish_cmd_vel(self.steer, velocity=0.5)
    def publish_cmd_vel(self, steering_angle, velocity = None, clip = True):
        """
        steering_angle: in degrees
        velocity: in m/s
        """
        if velocity is None:
            velocity = self.maxspeed
        if clip:
            steering_angle = np.clip(steering_angle, -23, 23)
        self.msg.data = '{"action":"1","speed":'+str(velocity)+'}'
        self.msg2.data = '{"action":"2","steerAngle":'+str(float(steering_angle))+'}'
        # print(self.msg.data)
        self.cmd_vel_pub.publish(self.msg)
        self.cmd_vel_pub.publish(self.msg2)
    def get_steering_angle(self, offset=0):
        """
        Determine the steering angle based on the lane center
        :param center: lane center
        :return: Steering angle in degrees
        """
        # Calculate the steering angle in radians
        self.dt = (rospy.Time.now()-self.timerpid).to_sec()
        self.timerpid = rospy.Time.now()
        error = (self.center + offset - self.image_center)
        try:
            d_error = (error-self.last)/self.dt
        except:
            return 0
        self.last = error
        steering_angle = np.clip((error*self.p+d_error*self.d)*180/np.pi, -23, 23)
        return steering_angle
    def update_states_rk4(self, speed, steering_angle):
        # dead reckoning odometry using Runge-Kutta 4th order method
        dt = (rospy.Time.now()-self.timerodom).to_sec()
        self.timerodom = rospy.Time.now()
        magnitude = speed * dt * self.odomRatio
        yaw_rate = magnitude * math.tan(-steering_angle*math.pi/180) / self.wheelbase
        
        k1_x = magnitude * math.cos(self.yaw)
        k1_y = magnitude * math.sin(self.yaw)
        k1_yaw = yaw_rate

        k2_x = magnitude * math.cos(self.yaw + dt/2 * k1_yaw)
        k2_y = magnitude * math.sin(self.yaw + dt/2 * k1_yaw)
        k2_yaw = yaw_rate

        k3_x = magnitude * math.cos(self.yaw + dt/2 * k2_yaw)
        k3_y = magnitude * math.sin(self.yaw + dt/2 * k2_yaw)
        k3_yaw = yaw_rate

        k4_x = magnitude * math.cos(self.yaw + dt * k3_yaw)
        k4_y = magnitude * math.sin(self.yaw + dt * k3_yaw)
        k4_yaw = yaw_rate

        dx = 1 / 6 * (k1_x + 2 * k2_x + 2 * k3_x + k4_x)
        dy = 1 / 6 * (k1_y + 2 * k2_y + 2 * k3_y + k4_y)
        dyaw = 1 / 6 * (k1_yaw + 2 * k2_yaw + 2 * k3_yaw + k4_yaw)
        # print(f"dt: {dt:.4f}, magnitude: {magnitude:.4f}, yaw: {self.yaw:.4f}, speed: {speed:.4f}, k1_x: {k1_x:.4f}, k2_x: {k2_x:.4f}, k3_x: {k3_x:.4f}, k4_x: {k4_x:.4f}")
        # print(f"dt: {dt:.4f}, magnitude: {magnitude:.4f}, yaw: {self.yaw:.4f}, speed: {speed:.4f}, dx: {dx:.4f}, dy: {dy:.4f}, dyaw: {dyaw:.4f}, k1_x: {k1_x:.4f}")
        return dx, dy, dyaw

    def update_states_euler(self, speed, steering_angle):
        dt = (rospy.Time.now()-self.timerodom).to_sec()
        self.timerodom = rospy.Time.now()
        magnitude = speed*dt*self.odomRatio

        odomYaw += magnitude*math.tan(-steering_angle*math.pi/180)/self.wheelbase
        odomYaw = np.fmod(odomYaw, 2*math.pi) 
        if odomYaw < 0: #convert to 0-2pi
            odomYaw += 2*np.pi
        self.odomX += magnitude * math.cos(self.yaw)
        self.odomY += magnitude * math.sin(self.yaw)
        return
    def publish_static_transforms(self):
        static_transforms = []

        t = TransformStamped()
        t.header.stamp = rospy.Time.now()

        # chassis -> camera
        t_camera = TransformStamped()
        t_camera.header.stamp = rospy.Time.now()
        t_camera.header.frame_id = "chassis"
        t_camera.child_frame_id = "camera"
        joint_x, joint_y, joint_z = 0, 0, 0
        link_x, link_y, link_z = 0, 0, 0.2
        t_camera.transform.translation.x = joint_x + link_x
        t_camera.transform.translation.y = joint_y + link_y
        t_camera.transform.translation.z = joint_z + link_z
        roll, pitch, yaw = 0, 0.2617, 0
        qtn = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        t_camera.transform.rotation.x = qtn[0]
        t_camera.transform.rotation.y = qtn[1]
        t_camera.transform.rotation.z = qtn[2]
        t_camera.transform.rotation.w = qtn[3]
        static_transforms.append(t_camera)

        # chassis -> laser
        t_laser = TransformStamped()
        t_laser.header.stamp = rospy.Time.now()
        t_laser.header.frame_id = "chassis"
        t_laser.child_frame_id = "laser"
        joint_x, joint_y, joint_z = 0, 0, 0
        link_x, link_y, link_z = 0, 0, 0.2
        t_laser.transform.translation.x = joint_x + link_x
        t_laser.transform.translation.y = joint_y + link_y
        t_laser.transform.translation.z = joint_z + link_z
        t_laser.transform.rotation.x = 0
        t_laser.transform.rotation.y = 0
        t_laser.transform.rotation.z = 0
        t_laser.transform.rotation.w = 1
        static_transforms.append(t_laser)

        # chassis -> imu
        t_laser = TransformStamped()
        t_laser.header.stamp = rospy.Time.now()
        t_laser.header.frame_id = "chassis"
        t_laser.child_frame_id = "imu0"
        joint_x, joint_y, joint_z = 0, 0, 0
        link_x, link_y, link_z = 0, 0, 0.0
        t_laser.transform.translation.x = joint_x + link_x
        t_laser.transform.translation.y = joint_y + link_y
        t_laser.transform.translation.z = joint_z + link_z
        t_laser.transform.rotation.x = 0
        t_laser.transform.rotation.y = 0
        t_laser.transform.rotation.z = 0
        t_laser.transform.rotation.w = 1
        static_transforms.append(t_laser)
        self.static_broadcaster.sendTransform(static_transforms)
