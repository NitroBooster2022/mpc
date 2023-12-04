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

class Utility {
public:
    Utility(ros::NodeHandle& nh_, bool subSign = true, bool useEkf = false, bool subLane = false,  bool subModel = true, bool subImu = true, bool pubOdom = true);
    ~Utility();
    void callTriggerService();
// private:
    ros::NodeHandle& nh;
    ros::ServiceClient triggerServiceClient;
    // Constants
    static const int NUM_VALUES_PER_OBJECT = 7;
    enum SignValues { x1, y1, x2, y2, distance, confidence, id };

    int num_obj;
    std::mutex lock;
    bool pubOdom, useIMU, subLane, subSign, subModel, subImu, useEkf;
    double rateVal;
    ros::Rate* rate;

    double wheelbase, odomRatio, maxspeed, center, image_center, p, d, last;
    double yaw, velocity, odomX, odomY, odomYaw, dx, dy, dyaw, ekf_x, ekf_y, ekf_yaw, gps_x, gps_y, steer_command, velocity_command, x_speed, y_speed;
    double gps_state[3];
    double ekf_state[3];
    std::optional<size_t> car_idx;

    ros::Time timerodom;
    std::optional<ros::Time> timerpid;
    std::optional<ros::Time> initializationTimer;
    ros::Time general_timer;

    double covariance_value;

    bool initializationFlag;

    tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::PoseWithCovarianceStamped pose_to_set;

    // publishers
    ros::Publisher odom_pub;
    ros::Publisher odom1_pub;
    ros::Publisher cmd_vel_pub;

    // messages
    nav_msgs::Odometry odom_msg;
    nav_msgs::Odometry odom1_msg;
    nav_msgs::Odometry ekf_msg;
    std_msgs::String msg;
    std_msgs::String msg2;

    gazebo_msgs::ModelStates model;
    std_msgs::Float32MultiArray sign;
    utils::Lane lane;
    sensor_msgs::Imu imu;
    tf2::Quaternion q;
    tf2::Matrix3x3 m;
    tf2::Quaternion tf2_quat;

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
    void process_yaw();
    void publish_odom();
    int object_index(int obj_id);
    double object_distance(int index);
    std::array<float, 4> object_box(int index);
    void set_initial_pose(double x, double y, double yaw);
    void update_states_rk4(double velocity, double steer, double dt=-1);
    geometry_msgs::TransformStamped add_static_link(double x, double y, double z, double roll, double pitch, double yaw, std::string parent, std::string child);
    void publish_cmd_vel(double steering_angle, double velocity = -3.57, bool clip = true);
    void lane_follow();
    void idle();
    double  get_steering_angle(double offset=0);
    void set_rate(double rateVal);
    double get_current_orientation();
    std::array<double, 3> get_real_states() const;
};