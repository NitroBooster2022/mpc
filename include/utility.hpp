#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
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
#include "constants.h"

using namespace VehicleConstants;

class Utility {
public:
    
    Utility(ros::NodeHandle& nh_, bool real, double x0, double y0, double yaw0, bool subSign = true, bool useEkf = false, bool subLane = false,  std::string robot_name = "car1", bool subModel = false, bool subImu = true, bool pubOdom = true);
    ~Utility();
    void callTriggerService();
// private:

    //tunables
    double left_trajectory1, left_trajectory2, right_trajectory1, right_trajectory2, p_rad;

    typedef double (Utility::*TrajectoryFunction)(double x);
    TrajectoryFunction trajectoryFunction;
    int intersectionDecision;
    ros::NodeHandle& nh;
    ros::ServiceClient triggerServiceClient;
    
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
    struct CameraPose {
        double x;
        double y;
        double z;
    } const CAMERA_POSE = {0, 0, 0.2};

    int num_obj = 0;
    std::mutex lock;
    bool pubOdom, useIMU, subLane, subSign, subModel, subImu, useEkf, hasGps;
    bool real;
    bool useGmapping = true, useLidarOdom = false, useAmcl = false;
    double rateVal;
    ros::Rate* rate;

    double wheelbase, odomRatio, maxspeed, center, image_center, p, d, last;
    bool stopline = false;
    double yaw, velocity, steer_command, velocity_command, x_speed, y_speed;
    double odomX, odomY, odomYaw, dx, dy, dyaw, ekf_x, ekf_y, ekf_yaw, gps_x, gps_y, gmapping_x, gmapping_y, odomX_lidar, odomY_lidar;
    double initial_yaw = 0;
    double x_offset, y_offset;
    double x0 = -1, y0 = -1, yaw0 = 0;
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
    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::Buffer tfBuffer;

    // publishers
    ros::Publisher odom_pub;
    ros::Publisher odom_lidar_pub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher car_pose_pub;
    ros::Publisher pose_pub;
    ros::Publisher waypoints_pub;
    ros::Publisher detected_cars_pub;
    ros::Publisher state_offset_pub;

    // messages
    nav_msgs::Odometry odom_msg;
    nav_msgs::Odometry odom_lidar_msg;
    // nav_msgs::Odometry odom1_msg;
    nav_msgs::Odometry ekf_msg;
    std_msgs::String msg;
    std_msgs::String msg2;
    std_msgs::Float32MultiArray car_pose_msg;
    std_msgs::Float32MultiArray state_offset_msg;

    gazebo_msgs::ModelStates model;
    std_msgs::Float32MultiArray sign;
    utils::Lane lane;
    sensor_msgs::Imu imu_msg;
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
    ros::Subscriber tf_sub;
    ros::Subscriber odom_lidar_sub;
    ros::Subscriber amcl_sub;

    ros::Timer odom_pub_timer;
    void odom_pub_timer_callback(const ros::TimerEvent&);
    ros::Timer imu_pub_timer;
    void imu_pub_timer_callback(const ros::TimerEvent&);
    ros::Publisher imu_pub;

    // Callbacks
    void lane_callback(const utils::Lane::ConstPtr& msg);
    void sign_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void model_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void imu_callback(const sensor_msgs::Imu::ConstPtr& msg);
    void ekf_callback(const nav_msgs::Odometry::ConstPtr& msg);
    void tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg);
    void amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void odom_lidar_callback(const nav_msgs::Odometry::ConstPtr& msg);
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
    double  get_steering_angle(double offset=-20);
    void set_rate(double rateVal);
    double get_current_orientation();
    std::array<double, 3> get_real_states() const;
    boost::asio::io_service io;
    // boost::asio::serial_port serial;
    std::unique_ptr<boost::asio::serial_port> serial;
    double get_yaw() {
        return yaw;
    }
    int get_states(double &x_, double &y_, double &yaw_) {
        // if(!initializationFlag || !imuInitialized || x0 < 0 || y0 < 0) {
        //     ROS_INFO("get_states: Initialization not done");
        //     return -1;
        // }
        yaw_ = yaw;
        if(useEkf) {
            // ROS_INFO("Using ekf: %.3f, %.3f", ekf_x, ekf_y);
            x_ = ekf_x;
            y_ = ekf_y;
        } else if(subModel) {
            // ROS_INFO("Using gps: %.3f, %.3f", gps_x, gps_y);
            x_ = gps_x;
            y_ = gps_y;
        } else if(useGmapping || useAmcl) {
            // ROS_INFO("Using gmapping: %.3f, %.3f", gmapping_x, gmapping_y);
            x_ = gmapping_x;
            y_ = gmapping_y;
        } else if(useLidarOdom) {
            // ROS_INFO("Using lidar odom: %.3f, %.3f", odomX_lidar, odomY_lidar);
            x_ = odomX_lidar + x0;
            y_ = odomY_lidar + y0;
        } else {
            // ROS_INFO("Using odom: %.3f, %.3f", odomX, odomY);
            x_ = odomX + x0;
            y_ = odomY + y0;
        }
        return 0;
    }
    int recalibrate_states(double x_offset, double y_offset) {
        if(useEkf) {
            if (hasGps) {
                ekf_x += x_offset;
                ekf_y += y_offset;
                set_pose_using_service(ekf_x, ekf_y, yaw);
            } else {
                x0 += x_offset;
                y0 += y_offset;
            }
        } else {
            x0 += x_offset;
            y0 += y_offset;
        }
    }
    int reinitialize_states() {
        if(useEkf) {
            std::cout << "waiting for ekf message" << std::endl;
            ros::topic::waitForMessage<nav_msgs::Odometry>("/odometry/filtered");
            std::cout << "received message from ekf" << std::endl;
            x0 = ekf_x;
            y0 = ekf_y;
        } else if(subModel) {
            std::cout << "waiting for model message" << std::endl;
            ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
            std::cout << "received message from model" << std::endl;
            x0 = gps_x;
            y0 = gps_y;
        } else if(useGmapping || useAmcl) {
            x0 = gmapping_x;
            y0 = gmapping_y;
        } else if(useLidarOdom) {
            x0 = odomX_lidar;
            y0 = odomY_lidar;
        } else {
            x0 = odomX;
            y0 = odomY;
        }
        return 1;
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
    
    // Eigen::Vector2d estimate_object_pose2d(double x, double y, double yaw, double x1, double y1, double x2, double y2, double object_distance, const std::array<double, 4>& camera_params, bool is_car = false) {
    Eigen::Vector2d estimate_object_pose2d(double x, double y, double yaw,
                                       double x1, double y1, double x2, double y2,
                                       double object_distance,
                                       const std::array<double, 4>& camera_params,
                                       bool is_car = false)
    {
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

        // std::array<double, 2> vehicle_pos = { x, y }; // Only x and y are needed for 2D
        
        // double fx = camera_params[0];
        // double cx = camera_params[2];
        
        // double bbox_center_x = (x1 + x2) / 2;
        
        // double X_c = (bbox_center_x - cx) / fx * object_distance;
        
        // // Vehicle coordinates (X_v is forward, Y_v is left/right from the vehicle's perspective)
        // double X_v = object_distance;
        // double Y_v = -X_c; 

        // std::array<std::array<double, 2>, 2> rotation_matrix = {{
        //     { std::cos(yaw), -std::sin(yaw) },
        //     { std::sin(yaw), std::cos(yaw) }
        // }};
        
        // std::array<double, 2> vehicle_coordinates = {
        //     rotation_matrix[0][0] * X_v + rotation_matrix[0][1] * Y_v,
        //     rotation_matrix[1][0] * X_v + rotation_matrix[1][1] * Y_v
        // };
        
        // Eigen::Vector2d world_coordinates = {
        //     vehicle_pos[0] + vehicle_coordinates[0],
        //     vehicle_pos[1] + vehicle_coordinates[1]
        // };
        
        // return world_coordinates;

        // Extract camera parameters
        double fx = camera_params[0];
        double fy = camera_params[1];
        double cx = camera_params[2];
        double cy = camera_params[3];

        // Compute bounding box center in image coordinates
        double bbox_center_x = (x1 + x2) / 2;
        double bbox_center_y = (y1 + y2) / 2;

        // Convert image coordinates to normalized coordinates
        double x_norm = (bbox_center_x - cx) / fx;
        double y_norm = (bbox_center_y - cy) / fy;

        // Estimate 3D coordinates in the camera frame
        double X_c = x_norm * object_distance;
        double Y_c = y_norm * object_distance;
        double Z_c = object_distance;

        // 3D point in the camera frame
        Eigen::Vector3d P_c(X_c, Y_c, Z_c);

        // Convert to vehicle coordinates (vehicle's x-axis is forward, y-axis is left/right)
        Eigen::Vector3d P_v(Z_c, -X_c, 0);

        // Rotation matrix from vehicle to world coordinates
        Eigen::Matrix2d R_vw;
        R_vw << std::cos(yaw), -std::sin(yaw),
                std::sin(yaw), std::cos(yaw);

        // Translate to world coordinates
        Eigen::Vector2d vehicle_pos(x, y);
        Eigen::Vector2d P_v_2d(P_v[0], P_v[1]);
        Eigen::Vector2d world_coordinates = vehicle_pos + R_vw * P_v_2d;

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
        return exp(left_trajectory1 * (x + left_trajectory2));
    }

    double rightTrajectorySim(double x) {
        return - exp(right_trajectory1 * (x + right_trajectory2));
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
        // static double p_rad = 0;
        // if (real) {
        //     p_rad = 3.25;
        // } else {
        //     p_rad = 2.35;
        // }
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

    void send_speed_and_steer(float f_velocity, float f_angle) {
        ROS_INFO("speed:%.3f, angle:%.3f, yaw:%.3f, odomX:%.2f, odomY:%.2f, ekfx:%.2f, ekfy:%.2f", f_velocity, f_angle, yaw * 180 / M_PI, odomX, odomY, ekf_x-x0, ekf_y-y0);
        if (serial == nullptr) {
            ROS_INFO("Serial is null");
            return;
        }
        std::stringstream strs;
        char buff[100];
        snprintf(buff, sizeof(buff), "%.2f:%.2f;;\r\n", f_velocity * 100, f_angle);
        strs << "#" << "8" << ":" << buff;
        boost::asio::write(*serial, boost::asio::buffer(strs.str()));
        // std::cout << strs.str() << std::endl;
    }

};
