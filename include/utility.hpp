#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <array>
#include <eigen3/Eigen/Dense>
#include "utils/Lane.h"
#include <std_srvs/Trigger.h>
#include <mutex>
#include <cmath>
#include <boost/asio.hpp>

class Utility {
public:
    
    Utility(ros::NodeHandle& nh_, bool real, double x0, double y0, double yaw0, bool subSign = true, bool useEkf = false, bool subLane = false,  std::string robot_name = "car1", bool subModel = false, bool subImu = true, bool pubOdom = true);
    ~Utility();
    void callTriggerService();
// private:
    typedef double (Utility::*TrajectoryFunction)(double x);
    TrajectoryFunction trajectoryFunction;
    int intersectionDecision;
    ros::NodeHandle& nh;
    ros::ServiceClient triggerServiceClient;
    // Constants
    static const int NUM_VALUES_PER_OBJECT = 7;
    enum SignValues { x1, y1, x2, y2, distance, confidence, id };
    enum LOCALIZATION_SOURCE {
        ODOM,
        EKF
    };
    static constexpr double CAM_TO_CAR_FRONT = 0.21;
    static constexpr double CAR_LENGTH = 0.464;
    static constexpr double CAR_WIDTH = 0.1885;
    static constexpr double CAR_HEIGHT = 0.1155;
    static constexpr double MAX_TAILING_DIST = 1.0;
    static constexpr double MIN_SIGN_DIST = 0.39;  // 0.6 - 0.21
    static constexpr double MAX_SIGN_DIST = 1.09;  // 1.3 - 0.21
    static constexpr double MAX_PARK_DIST = 0.79;  // 1.0 - 0.21
    static constexpr double PARKSIGN_TO_CAR = 0.51;
    static constexpr double PARK_OFFSET = 1.31;    // 1.1 + 0.21
    static constexpr double PARKING_SPOT_LENGTH = 0.723;
    static constexpr double OVERTAKE_DIST = 2.0;
    static constexpr double LANE_OFFSET = 0.36;
    static constexpr double MIN_DIST_TO_CAR = 0.8;
    static constexpr double MAX_CAR_DIST = 3.0;
    static constexpr double SIGN_COOLDOWN = 1.0;
    static constexpr double TOLERANCE_SQUARED = 0.01;
    static constexpr double STOP_DURATION = 1.50;
    
    std::string robot_name;
    // std::vector<std::array<double, 2>> detected_cars;
    std::vector<Eigen::Vector2d> detected_cars;
    std::vector<int> detected_cars_counter;
    std::list<int> recent_car_indices;
    void print_detected_cars() {
        for (size_t i = 0; i < detected_cars.size(); i++) {
            std::cout << "Car " << i << ": " << detected_cars[i][0] << ", " << detected_cars[i][1] << std::endl;
        }
    }
    std::array<double, 4> CAMERA_PARAMS = {554.3826904296875, 554.3826904296875, 320, 240};
    struct CameraPose {
        double x;
        double y;
        double z;
    } const CAMERA_POSE = {0, 0, 0.2};

    int num_obj;
    std::mutex lock;
    bool pubOdom, useIMU, subLane, subSign, subModel, subImu, useEkf;
    bool real;
    double rateVal;
    ros::Rate* rate;

    double wheelbase, odomRatio, maxspeed, center, image_center, p, d, last;
    bool stopline = false;
    double yaw, velocity, odomX, odomY, odomYaw, dx, dy, dyaw, ekf_x, ekf_y, ekf_yaw, gps_x, gps_y, steer_command, velocity_command, x_speed, y_speed;
    double initial_yaw = 0;
    double x_offset, y_offset;
    double gps_state[3];
    double ekf_state[3];
    std::optional<size_t> car_idx;

    ros::Time timerodom;
    std::optional<ros::Time> timerpid;
    std::optional<ros::Time> initializationTimer;
    ros::Time general_timer;

    double covariance_value;

    bool initializationFlag, imuInitialized = false;

    tf2_ros::StaticTransformBroadcaster static_broadcaster;

    // publishers
    ros::Publisher odom_pub;
    // ros::Publisher odom1_pub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher car_pose_pub;

    // messages
    nav_msgs::Odometry odom_msg;
    // nav_msgs::Odometry odom1_msg;
    nav_msgs::Odometry ekf_msg;
    std_msgs::String msg;
    std_msgs::String msg2;
    std_msgs::Float64MultiArray car_pose_msg;

    gazebo_msgs::ModelStates model;
    std_msgs::Float32MultiArray sign;
    utils::Lane lane;
    sensor_msgs::Imu imu;
    tf2::Quaternion q_imu;
    tf2::Matrix3x3 m_chassis;
    tf2::Quaternion tf2_quat;
    tf2::Quaternion q_transform;
    tf2::Quaternion q_chassis;

    // subscribers
    ros::Subscriber lane_sub;
    ros::Subscriber sign_sub;
    std::vector<float> detected_objects;
    ros::Subscriber model_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber ekf_sub;

    ros::Timer odom_pub_timer;
    void odom_pub_timer_callback(const ros::TimerEvent&);

    // Callbacks
    void lane_callback(const utils::Lane::ConstPtr& msg);
    void sign_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void model_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    void ekf_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void spin();
    // Methods
    void stop_car();
    void publish_static_transforms();
    void set_pose_using_service(double x, double y, double yaw);
    void publish_odom();
    int object_index(int obj_id);
    std::vector<int> object_indices(int obj_id);
    double object_distance(int index);
    std::array<double, 4> object_box(int index);
    void object_box(int index, std::array<double, 4>& oBox);
    void set_initial_pose(double x, double y, double yaw);
    void reset_odom();
    void update_states_rk4(double velocity, double steer, double dt=-1);
    geometry_msgs::TransformStamped add_static_link(double x, double y, double z, double roll, double pitch, double yaw, std::string parent, std::string child);
    void publish_cmd_vel(double steering_angle, double velocity = -3.57, bool clip = true);
    void lane_follow();
    void idle();
    double  get_steering_angle(double offset=0);
    void set_rate(double rateVal);
    double get_current_orientation();
    std::array<double, 3> get_real_states() const;
    boost::asio::io_service io;
    // boost::asio::serial_port serial;
    std::unique_ptr<boost::asio::serial_port> serial;
    double get_yaw() {
        return yaw;
    }
    void get_states(double &x_, double &y_, double &yaw_) {
        yaw_ = yaw;
        if(useEkf) {
            x_ = ekf_x;
            y_ = ekf_y;
        } else if(subModel) {
            x_ = gps_x;
            y_ = gps_y;
        } else {
            x_ = odomX;
            y_ = odomY;
        }
    }
    void get_gps_states(double &x_, double &y_, double &yaw_) {
        x_ = gps_x;
        y_ = gps_y;
        yaw_ = yaw;
    }
    void get_ekf_states(double &x_, double &y_, double &yaw_) {
        x_ = ekf_x;
        y_ = ekf_y;
        yaw_ = yaw;
    }
    
    Eigen::Vector2d estimate_object_pose2d(double x, double y, double yaw, double x1, double y1, double x2, double y2, double object_distance, const std::array<double, 4>& camera_params, bool is_car = false) {
        static double parallel_w2h_ratio = 1.30;
        static double perpendicular_w2h_ratio = 2.70;

        if (is_car) {
            double car_pixel_w2h_ratio = std::abs((x2 - x1) / (y2 - y1));
            // std::cout << "car_pixel_w2h_ratio: " << car_pixel_w2h_ratio << std::endl;

            // Normalize the ratio to a scale of 0 (parallel) to 1 (perpendicular)
            double normalized_ratio_parallel = std::max((car_pixel_w2h_ratio / parallel_w2h_ratio), 1.0);
            double normalized_ratio_perpendicular = std::min(car_pixel_w2h_ratio / perpendicular_w2h_ratio, 1.0);
            // std::cout << "normalized_ratio_parallel: " << normalized_ratio_parallel << std::endl;
            // std::cout << "normalized_ratio_perpendicular: " << normalized_ratio_perpendicular << std::endl;

            double parallel_diff = std::abs(normalized_ratio_parallel - 1);
            double perpendicular_diff = std::abs(normalized_ratio_perpendicular - 1);
            double dist;
            if (car_pixel_w2h_ratio < 2.0 || parallel_diff < perpendicular_diff) { // Parallel to the camera
                // std::cout << "Parallel to the camera" << std::endl;
                dist = CAR_LENGTH / 2 / normalized_ratio_parallel;
            } else { // Perpendicular to the camera
                dist = CAR_WIDTH / 2 / normalized_ratio_perpendicular;
            }
            
            object_distance += dist;
        }

        std::array<double, 2> vehicle_pos = { x, y }; // Only x and y are needed for 2D
        
        double fx = camera_params[0];
        double cx = camera_params[2];
        
        double bbox_center_x = (x1 + x2) / 2;
        
        double X_c = (bbox_center_x - cx) / fx * object_distance;
        
        // Vehicle coordinates (X_v is forward, Y_v is left/right from the vehicle's perspective)
        double X_v = object_distance;
        double Y_v = -X_c; 

        std::array<std::array<double, 2>, 2> rotation_matrix = {{
            { std::cos(yaw), -std::sin(yaw) },
            { std::sin(yaw), std::cos(yaw) }
        }};
        
        std::array<double, 2> vehicle_coordinates = {
            rotation_matrix[0][0] * X_v + rotation_matrix[0][1] * Y_v,
            rotation_matrix[1][0] * X_v + rotation_matrix[1][1] * Y_v
        };
        
        Eigen::Vector2d world_coordinates = {
            vehicle_pos[0] + vehicle_coordinates[0],
            vehicle_pos[1] + vehicle_coordinates[1]
        };
        
        return world_coordinates;
    }
    Eigen::Vector2d estimate_object_pose2d(double x, double y, double yaw, const std::array<double, 4>& bounding_box, double object_distance, const std::array<double, 4>& camera_params, bool is_car = false) {
        double x1 = bounding_box[0];
        double y1 = bounding_box[1];
        double x2 = bounding_box[2];
        double y2 = bounding_box[3];
        return estimate_object_pose2d(x, y, yaw, x1, y1, x2, y2, object_distance, camera_params, is_car);
    }
    // 8773598130 2036 0590595 
    static std::string getSourceDirectory() {
        std::string file_path(__FILE__); 
        size_t last_dir_sep = file_path.rfind('/');
        if (last_dir_sep == std::string::npos) {
            last_dir_sep = file_path.rfind('\\'); 
        }
        if (last_dir_sep != std::string::npos) {
            return file_path.substr(0, last_dir_sep);  // Extract directory path
        }
        return "";  // Return empty string if path not found
    }
    double straightTrajectory(double x) {
        return 0;
    }

    double leftTrajectorySim(double x) {
        if (real) {
            return -exp(3.57 * x - 3.9);
        }
        return exp(3.57 * x - 4.2);
    }

    double rightTrajectorySim(double x) {
        if (real) {
            return -exp(3.75 * (x - 0.49));
        }
        return -exp(3.75 * x - 3.);
    }
    void setIntersectionDecision(int decision) {
        intersectionDecision = decision;
        std::cout << "Intersection decision: " << intersectionDecision << std::endl;
        switch (decision) {
            case 0: // Left
                trajectoryFunction = &Utility::leftTrajectorySim;
                ROS_INFO("Left trajectory");
                break;
            case 1: // Straight
                trajectoryFunction = &Utility::straightTrajectory;
                ROS_INFO("Straight trajectory");
                break;
            case 2: // Right
                trajectoryFunction = &Utility::rightTrajectorySim;
                ROS_INFO("Right trajectory");
                break;
            default:
                trajectoryFunction = nullptr;
                ROS_INFO("Invalid trajectory");
        }
    }
    double computeTrajectory(double x) {
        if (trajectoryFunction != nullptr) {
            return (this->*trajectoryFunction)(x);
        }
        return 0;
    }
    double computeTrajectoryPid(double error) {
        static double last_error = 0;
        static double error_sum = 0;
        static ros::Time last_time = ros::Time::now() - ros::Duration(0.1);
        static double p_rad = 0;
        if (real) {
            p_rad = 3;
        } else {
            p_rad = 2.35;
        }
        static double p = p_rad * 180 / M_PI; //2.35
        static double d = 0;//1 * 180 / M_PI;
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;
        double derivative = (error - last_error) / dt;
        return p * error + d * derivative;
    }
    void send_speed(float f_velocity) {
        if (serial == nullptr) {
            ROS_INFO("Serial is null");
            return;
        }
        std::stringstream strs;
        char buff[100];
        snprintf(buff, sizeof(buff), "%.2f;;\r\n", f_velocity * 100);
        strs << "#" << "1" << ":" << buff;
        boost::asio::write(*serial, boost::asio::buffer(strs.str()));
    }

    void send_steer(float f_angle) {
        if (serial == nullptr) {
            ROS_INFO("Serial is null");
            return;
        }
        std::stringstream strs;
        char buff[100];
        snprintf(buff, sizeof(buff), "%.2f;;\r\n", f_angle);
        strs << "#" << "2" << ":" << buff;
        boost::asio::write(*serial, boost::asio::buffer(strs.str()));
    }
};
