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
from utils.msg import IMU, Lane, Sign
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from std_msgs.msg import Float32MultiArray

class Utility:
    def __init__(self, lock, useIMU=True, subLane=False, subSign=False, subModel=False, subImu=False, pubOdom=False, useEkf=False):
        rospy.on_shutdown(self.stop_car)
        self.lock = lock
        self.rateVal = 50.0
        self.rate = rospy.Rate(self.rateVal)
        self.pubOdom = pubOdom # publish odom in imu callback if true
        self.useEkf = useEkf
        self.useIMU = useIMU
        self.process_yaw = self.process_IMU if useIMU else self.process_Imu

        # Constants
        self.NUM_VALUES_PER_OBJECT = 7
        self.SIGN_VALUES = {"x1": 0, "y1": 1, "x2": 2, "y2": 3, "distance": 4, "confidence": 5, "id": 6}

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
        self.ekf_x = 0.0
        self.ekf_y = 0.0
        self.gps_x = 0.0
        self.gps_y = 0.0
        self.steer_command = 0.0
        self.velocity_command = 0.0
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

        covariance_value = 0.01
        covariance_matrix = (np.identity(6)*covariance_value).flatten().tolist()
        # covariance_matrix = [covariance_value] * 6 + [0] * 30
        # Publishers
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=3)
        self.odom_msg = Odometry()
        self.odom_msg.pose.covariance = covariance_matrix
        self.odom_msg.twist.covariance = covariance_matrix

        self.odom1_pub = rospy.Publisher("odom1", Odometry, queue_size=3)
        self.odom1_msg = Odometry()
        covariance_matrix = [0.01] * 6 + [0] * 30
        self.odom1_msg.pose.covariance = covariance_matrix
        self.odom1_msg.twist.covariance = covariance_matrix

        self.cmd_vel_pub = rospy.Publisher("/automobile/command", String, queue_size=3)
        self.msg = String()
        self.msg2 = String()

        print("waiting for IMU message")
        if self.useIMU:
            rospy.wait_for_message("/automobile/IMU", IMU)
        else:
            rospy.wait_for_message("/camera/imu", Imu)
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
        if subSign:
            # self.numObj = -1
            # self.detected_objects = []
            # self.box1 = []
            # self.box2 = []
            # self.box3 = []
            # self.confidence = []
            # self.distances = []
            # self.min_sizes = [25,25,60,50,45,35,30,25,25,130,75,72,70]
            # self.max_sizes = [100,75,125,100,120,125,70,75,100,350,170,250,320]
            # self.sign_sub = rospy.Subscriber("/sign", Sign, self.sign_callback, queue_size=3)
            self.detected_objects = np.array([])
            self.numObj = -1
            self.sign_sub = rospy.Subscriber("/sign", Float32MultiArray, self.sign_callback, queue_size=3)
            self.sign = Sign()
            print("waiting for sign message")
            rospy.wait_for_message("/sign", Sign)
            print("received message from sign")
        if useEkf:
            self.ekf_sub = rospy.Subscriber("/odometry/filtered", Odometry, self.ekf_callback, queue_size=3)
            self.ekf_msg = Odometry()

    # Callbacks
    def sign_callback(self, sign):
        with self.lock:
            if sign.data:
                self.numObj = len(sign.data) // self.NUM_VALUES_PER_OBJECT
                if self.numObj == 1:
                    self.detected_objects = np.array(sign.data)
                    return 
                self.detected_objects = np.array(sign.data).reshape(-1, self.NUM_VALUES_PER_OBJECT).T 
            else:
                self.numObj = 0

    def lane_callback(self, lane):
        with self.lock:
            self.center = lane.center
    def imu_callback(self, imu):
        with self.lock:
            self.imu = imu
        if self.pubOdom: #if true publish odom in imu callback
            self.publish_odom()
    def ekf_callback(self, ekf):
        with self.lock:
            self.ekf_x = ekf.pose.pose.position.x
            self.ekf_y = ekf.pose.pose.position.y
    def model_callback(self, model):
        with self.lock:
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
            rospy.wait_for_service('/set_pose')  
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
            self.set_initial_pose(self.gps_x, self.gps_y, self.yaw)
            print(f"odomX: {self.odomX:.2f}, odomY: {self.odomY:.2f}")
            if self.useEkf:
                self.set_pose_using_service(self.odomX, self.odomY, self.yaw)
            return
        dx, dy, dyaw = self.update_states_rk4(self.velocity, self.steer_command)
        self.odomX += dx
        self.odomY += dy

        # print(f"dt: {dt:.2f}, odomx: {self.odomX:.2f}, odomy: {self.odomY:.2f}, gps_x: {self.gps_x:.2f}, gps_y: {self.gps_y:.2f}, x_error: {abs(self.odomX-self.gps_x):.2f}, y_error: {abs(self.odomY-self.gps_y):.2f}, yaw: {self.yaw:.2f}, steer: {self.steer_command:.2f}, speed: {self.velocity:.2f}")
       
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

        # Publish odom1
        self.odom1_msg.header.stamp = rospy.Time.now()
        self.odom1_msg.header.frame_id = "odom"
        self.odom1_msg.child_frame_id = "chassis"
        self.odom1_msg.twist.twist.linear.x = self.velocity*math.cos(self.yaw)
        self.odom1_msg.twist.twist.linear.y = self.velocity*math.sin(self.yaw)
        self.odom1_pub.publish(self.odom1_msg)
        return

    def get_real_states(self):
        return np.array([self.gps_x, self.gps_y, self.yaw])
    def set_initial_pose(self, x, y, yaw):
        self.odomX = x
        self.odomY = y
        self.odomYaw = yaw
    def lane_follow(self):
        self.steer_command = self.get_steering_angle()
        self.publish_cmd_vel(self.steer_command, velocity=0.5)
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
    def idle(self):
        self.steer_command = 0
        self.velocity_command = 0
        self.publish_cmd_vel(0, 0)
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
    def object_index(self, obj_id):
        """
        return index of object with id obj_id if it is detected, otherwise return -1
        """
        if self.numObj < 1:
            return -1
        if self.numObj == 1:
            if self.detected_objects[self.SIGN_VALUES["id"]]==obj_id:
                return 0
            return -1
        id_array = self.detected_objects[self.SIGN_VALUES["id"]]
        if obj_id in id_array:
            obj_index = np.where(id_array == obj_id)[0][0]
            return obj_index
        return -1
    def object_distance(self, index):
        if self.numObj == 1:
            return self.detected_objects[self.SIGN_VALUES["distance"]]
        return self.detected_objects[self.SIGN_VALUES["distance"], index]
    def object_box(self, index):
        if self.numObj == 1:
            return self.detected_objects[self.SIGN_VALUES["x1"]:self.SIGN_VALUES["y2"]+1]
        return self.detected_objects[self.SIGN_VALUES["x1"]:self.SIGN_VALUES["y2"]+1, index]
    # def object_distance(self, obj_id):
    #     """
    #     return distance to object with id obj_id if it is detected, otherwise return -1
    #     """
    #     # print("num: ", self.numObj)
    #     if self.numObj < 1:
    #         return -1
    #     if self.numObj == 1:
    #         # print("id: ", self.detected_objects[self.SIGN_VALUES["id"]])
    #         if self.detected_objects[self.SIGN_VALUES["id"]]==obj_id:
    #             # print("distance: ", self.detected_objects[self.SIGN_VALUES["distance"]])
    #             return self.detected_objects[self.SIGN_VALUES["distance"]]
    #         return -1
    #     id_array = self.detected_objects[self.SIGN_VALUES["id"]]
    #     # print("id_array: ", id_array)
    #     if obj_id in id_array:
    #         obj_index = np.where(id_array == obj_id)[0][0] # if multiple objects with same id, return the first one
    #         # print("obj_index: ", obj_index)
    #         # print("distance: ", self.detected_objects[self.SIGN_VALUES["distance"], obj_index])
    #         return self.detected_objects[self.SIGN_VALUES["distance"], obj_index]
    #     return -1
    # def object_box(self, obj_id):
    #     """
    #     return box of object with id obj_id if it is detected, otherwise return None
    #     """
    #     if self.numObj < 1:
    #         return None
    #     if self.numObj == 1:
    #         if self.detected_objects[self.SIGN_VALUES["id"]]==obj_id:
    #             return self.detected_objects[self.SIGN_VALUES["x1"]:self.SIGN_VALUES["y2"]+1]
    #         return None
    #     id_array = self.detected_objects[self.SIGN_VALUES["id"]]
    #     if obj_id in id_array:
    #         obj_index = np.where(id_array == obj_id)[0][0]
    #         return self.detected_objects[self.SIGN_VALUES["x1"]:self.SIGN_VALUES["y2"]+1, obj_index]
    
    def check_size(self, obj_id, index):
        #checks whether a detected object is within a certain min and max sizes defined by the obj type
        #box[0] is x, box[1] is y, box[2] is width, box[3] is height
        box = self.box1 if index==0 else self.box2
        conf = self.confidence[index]
        size = max(box[2], box[3]) #width or height
        if obj_id==12:
            size = min(box[2], box[3])
        if obj_id==10:
            conf_thresh = 0.35
        else:
            conf_thresh = 0.8
        return size >= self.min_sizes[obj_id] and size <= self.max_sizes[obj_id] and conf >= conf_thresh #check this
    def get_current_orientation(self):
        hsy = 0
        while self.yaw > 2 * np.pi:
            self.yaw -= 2 * np.pi
            hsy += 1
        while self.yaw < 0:
            self.yaw += 2 * np.pi
            hsy -= 1
        ori = np.argmin([abs(self.yaw),abs(self.yaw-np.pi/2),abs((self.yaw)-np.pi),abs(self.yaw-np.pi/2*3),abs(self.yaw-np.pi*2)])%4*np.pi/2
        ori += hsy * np.pi * 2
        return ori
    def set_rate(self, rate):
        self.rateVal = rate
        self.rate = rospy.Rate(self.rateVal)
    def call_trigger_service(self):
        try:
            trigger_service = rospy.ServiceProxy('trigger_service', Trigger)
            response = trigger_service()
            rospy.loginfo("Service response: %s", response.message)
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
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
