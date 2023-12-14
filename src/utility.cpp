#include <ros/ros.h>
#include "utility.hpp"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/impl/utils.h>
#include <vector>
#include <array>
#include <eigen3/Eigen/Dense>
#include "utils/Lane.h"
#include <std_srvs/Trigger.h>
#include <mutex>
#include <cmath>

Utility::Utility(ros::NodeHandle& nh_, bool subSign, bool useEkf, bool subLane, bool subModel, bool subImu, bool pubOdom) 
    : nh(nh_), useIMU(useIMU), subLane(subLane), subSign(subSign), subModel(subModel), subImu(subImu), pubOdom(pubOdom), useEkf(useEkf)
{
    nh.getParam("/x_offset", x_offset);
    nh.getParam("/y_offset", y_offset);
    rateVal = 10;
    rate = new ros::Rate(rateVal);
    wheelbase = 0.27;
    odomRatio = 1.0;
    maxspeed = 1.5;
    center = -1;
    image_center = 320;
    p = 0.005;
    d = 0.0005;
    last = 0;
    
    triggerServiceClient = nh_.serviceClient<std_srvs::Trigger>("trigger_service");
    static_broadcaster = tf2_ros::StaticTransformBroadcaster();
    publish_static_transforms();

    yaw = 1.5707;
    velocity = 0.0;
    odomX = 0.0;
    odomY = 0.0;
    odomYaw = 0.0;
    ekf_x = 0.0;
    ekf_y = 0.0;
    gps_x = 0.0;
    gps_y = 0.0;
    steer_command = 0.0;
    velocity_command = 0.0;
    // car_idx = std::nullopt;

    timerodom = ros::Time::now();
    initializationTimer = std::nullopt;
    timerpid = std::nullopt;

    initializationFlag = false;

    pose_to_set.header = std_msgs::Header();
    pose_to_set.header.frame_id = "chassis";

    covariance_value = 0.01 * 4;
    std::fill(std::begin(odom_msg.pose.covariance), std::end(odom_msg.pose.covariance), 0.0);
    std::fill(std::begin(odom1_msg.pose.covariance), std::end(odom1_msg.pose.covariance), 0.0);
    std::fill(std::begin(odom_msg.twist.covariance), std::end(odom_msg.twist.covariance), 0.0);
    std::fill(std::begin(odom1_msg.twist.covariance), std::end(odom1_msg.twist.covariance), 0.0);
    for (int hsy=0; hsy<36; hsy+=7) {
        odom_msg.pose.covariance[hsy] = covariance_value;
        odom1_msg.pose.covariance[hsy] = covariance_value;
        odom_msg.twist.covariance[hsy] = covariance_value;
        odom1_msg.twist.covariance[hsy] = covariance_value;
    }

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 3);
    odom1_pub = nh.advertise<nav_msgs::Odometry>("odom1", 3);
    cmd_vel_pub = nh.advertise<std_msgs::String>("/automobile/command", 3);

    std::cout << "waiting for Imu message" << std::endl;
    ros::topic::waitForMessage<sensor_msgs::Imu>("/camera/imu");
    std::cout << "waiting for model_states message" << std::endl;
    ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
    std::cout << "received message from Imu and model_states" << std::endl;

    if (useEkf) {
        ekf_sub = nh.subscribe("/odometry/filtered", 3, &Utility::ekf_callback, this);
        std::cout << "waiting for ekf message" << std::endl;
        ros::topic::waitForMessage<nav_msgs::Odometry>("/odometry/filtered");
        std::cout << "received message from ekf" << std::endl;
    } 
    if (subModel) {
        model_sub = nh.subscribe("/gazebo/model_states", 3, &Utility::model_callback, this);
    }
    if (subImu) {
        imu_sub = nh.subscribe("/camera/imu", 3, &Utility::imu_callback, this);
    }
    if (subLane) {
        lane_sub = nh.subscribe("/lane", 3, &Utility::lane_callback, this);
        std::cout << "waiting for lane message" << std::endl;
        ros::topic::waitForMessage<utils::Lane>("/lane");
        std::cout << "received message from lane" << std::endl;
        timerpid = ros::Time::now();
    }

    if (subSign) {
        sign_sub = nh.subscribe("/sign", 3, &Utility::sign_callback, this);
        std::cout << "waiting for sign message" << std::endl;
        ros::topic::waitForMessage<std_msgs::Float32MultiArray>("/sign");
        std::cout << "received message from sign" << std::endl;
    }
    if (pubOdom) {
        double odom_publish_frequency = rateVal; 
        // odom_pub_timer = nh.createTimer(ros::Duration(1.0 / odom_publish_frequency), &Utility::odom_pub_timer_callback, this);
    }
}

Utility::~Utility() {
    stop_car(); 
}

void Utility::odom_pub_timer_callback(const ros::TimerEvent&) {
    publish_odom();
}
void Utility::sign_callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    lock.lock();
    if (msg->data.size()) {
        num_obj = msg->data.size() / NUM_VALUES_PER_OBJECT;
        detected_objects.assign(msg->data.begin(), msg->data.end());
    } else {
        num_obj = 0;
    }
    lock.unlock();
}
void Utility::lane_callback(const utils::Lane::ConstPtr& msg) {
    lock.lock();
    center = msg->center;
    lock.unlock();
}
void Utility::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    // ros::Time now = ros::Time::now();
    lock.lock();
    // imu = *msg;
    q = tf2::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    m = tf2::Matrix3x3(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);
    // ROS_INFO("yaw: %3f", yaw);
    if (!imuInitialized) {
        imuInitialized = true;
        std::cout << "imu initialized" << std::endl;
    }
    lock.unlock();
    // ROS_INFO("imu time: %f", (now - ros::Time::now()).toSec());
    // ROS_INFO("imu callback rate: %3f", 1 / (now - general_timer).toSec());
    // general_timer = now;
}
void Utility::ekf_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    // ros::Time now = ros::Time::now();
    lock.lock();
    ekf_x = msg->pose.pose.position.x;
    ekf_y = msg->pose.pose.position.y;
    tf2::fromMsg(msg->pose.pose.orientation, tf2_quat);
    ekf_yaw = tf2::impl::getYaw(tf2_quat);
    lock.unlock();
    // ROS_INFO("ekf callback rate: %f", 1 / (now - general_timer).toSec());
    // general_timer = now;
}
void Utility::model_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // static ros::Time last_time;
    // ros::Time now = ros::Time::now();
    // double dt = (now - last_time).toSec();
    // last_time = now;
    // ROS_INFO("model callback rate: %3f", 1 / dt);

    lock.lock();
    // auto start = std::chrono::high_resolution_clock::now();
    if (!car_idx.has_value()) {
        auto it = std::find(msg->name.begin(), msg->name.end(), "automobile");
        if (it != msg->name.end()) {
            car_idx = std::distance(msg->name.begin(), it);
            std::cout << "automobile found: " << *car_idx << std::endl;
        } else {
            printf("automobile not found\n");
            lock.unlock();
            return; 
        }
    }
    
    auto& car_inertial = msg->twist[*car_idx];
    x_speed = msg->twist[*car_idx].linear.x;
    y_speed = msg->twist[*car_idx].linear.y;
    gps_x = msg->pose[*car_idx].position.x + x_offset;
    gps_y = msg->pose[*car_idx].position.y + y_offset;
    if (!initializationFlag && imuInitialized) {
        initializationFlag = true;
        std::cout << "Initializing... gps_x: " << gps_x << ", gps_y: " << gps_y << std::endl;
        set_initial_pose(gps_x, gps_y, yaw);
        std::cout << "odomX: " << odomX << ", odomY: " << odomY << std::endl;
        timerodom = ros::Time::now();
        // if (useEkf) set_pose_using_service(gps_x, gps_y, yaw);
        lock.unlock();
        return;
    }
    // ROS_INFO("gps_x: %3f, gps_y: %3f", gps_x, gps_y); // works
    // model = *msg;
    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> elapsed = end - start;
    // ROS_INFO("model callback time elapsed: %fs", elapsed.count());
    lock.unlock();
}

void Utility::stop_car() {
    std::cout << "Stopping car" << std::endl;
    msg.data = "{\"action\":\"1\",\"speed\":" + std::to_string(0.0) + "}";
    msg2.data = "{\"action\":\"2\",\"steerAngle\":" + std::to_string(0.0) + "}";
    for (int i = 0; i < 10; i++) {
        cmd_vel_pub.publish(msg2);
        cmd_vel_pub.publish(msg);
        // ros::Duration(0.1).sleep();
    }
    std::cout << "sent commands to stop car" << std::endl;
}

void Utility::set_pose_using_service(double x, double y, double yaw) {
    pose_to_set.pose.pose.position.x = x;
    pose_to_set.pose.pose.position.y = y;
    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    geometry_msgs::Quaternion ros_quaternion;
    ros_quaternion.x = q.x();
    ros_quaternion.y = q.y();
    ros_quaternion.z = q.z();
    ros_quaternion.w = q.w();
    if (useEkf) {
        std::cout << "waiting for set_pose service" << std::endl;
        ros::service::waitForService("/set_pose");
    }
    try {
        ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/set_pose");
        std_srvs::Trigger srv;
        client.call(srv);
    } catch (ros::Exception& e) {
        std::cout << "Service call failed: " << e.what() << std::endl;
    }
}

void Utility::process_yaw() {
    q = tf2::Quaternion(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
    m = tf2::Matrix3x3(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw);
}

void Utility::publish_odom() {
    // calculate rate
    // static ros::Time last_time;
    // ros::Time now = ros::Time::now();
    // double dt = (now - last_time).toSec();
    // last_time = now;
    // ROS_INFO("odom callback rate: %3f", 1 / dt);

    // calculate time elapsed
    // auto start = std::chrono::high_resolution_clock::now();
    
    // if (!car_idx.has_value()) {
    //     auto it = std::find(model.name.begin(), model.name.end(), "automobile");
    //     if (it != model.name.end()) {
    //         car_idx = std::distance(model.name.begin(), it);
    //         std::cout << "automobile found: " << *car_idx << std::endl;
    //     } else {
    //         return; 
    //     }
    // }

    if (!initializationFlag) {
        // initializationFlag = true;
        // std::cout << "Initializing... gps_x: " << gps_x << ", gps_y: " << gps_y << std::endl;
        // set_initial_pose(gps_x, gps_y, yaw);
        // std::cout << "odomX: " << odomX << ", odomY: " << odomY << std::endl;
        // timerodom = ros::Time::now();
        ROS_WARN("not initialized");
        return;
    }
    
    // process_yaw();
    yaw = fmod(yaw, 2 * M_PI);

    // Set velocity
    // auto& car_inertial = model.twist[*car_idx];
    // x_speed = car_inertial.linear.x;
    // y_speed = car_inertial.linear.y;
    double speed = sqrt(x_speed * x_speed + y_speed * y_speed);
    if (velocity_command < -0.01) {
        speed *= -1;
    }
    velocity = speed; 
    // ROS_INFO("speed: %3f, command: %3f", speed, velocity_command);

    // Set GPS
    // gps_x = model.pose[*car_idx].position.x; 
    // gps_y = 15 + model.pose[*car_idx].position.y;
    // ROS_INFO("gps_x: %3f, gps_y: %3f", gps_x, gps_y); // works

    // update_states_rk4(velocity, steer_command);
    update_states_rk4(velocity_command, steer_command);
    odomX += dx;
    odomY += dy;
    // ROS_INFO("odomX: %3f, gps_x: %3f, odomY: %3f, gps_y: %3f, error: %3f", odomX, gps_x, odomY, gps_y, sqrt((odomX - gps_x) * (odomX - gps_x) + (odomY - gps_y) * (odomY - gps_y))); // works

    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "chassis";
    odom_msg.pose.pose.position.x = odomX;
    odom_msg.pose.pose.position.y = odomY;
    odom_msg.pose.pose.position.z = 0.032939;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
    odom_msg.pose.pose.orientation = tf2::toMsg(quaternion);

    odom_msg.twist.twist.linear.x = velocity * cos(yaw);
    odom_msg.twist.twist.linear.y = velocity * sin(yaw);
    odom_pub.publish(odom_msg);

    // Prepare and publish odom1 message
    odom1_msg.header.stamp = ros::Time::now();
    odom1_msg.header.frame_id = "odom";
    odom1_msg.child_frame_id = "chassis";
    odom1_msg.twist.twist.linear.x = velocity * cos(yaw);
    odom1_msg.twist.twist.linear.y = velocity * sin(yaw);
    odom1_pub.publish(odom1_msg);
    // auto end = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double> elapsed = end - start;
    // ROS_INFO("time elapsed: %3fs", elapsed.count());
}

int Utility::object_index(int obj_id) {
    if (num_obj < 1) {
        return -1;
    }
    if (num_obj == 1) {
        if (detected_objects[id] == obj_id) {
            return 0;
        }
        return -1;
    }
    for (int i = 0; i < num_obj; ++i) {
        if (detected_objects[i * NUM_VALUES_PER_OBJECT + id] == obj_id) {
            return i;
        }
    }
    return -1;
}

double Utility::object_distance(int index) {
    if (num_obj == 1) {
        return detected_objects[distance];
    } else if (index >= 0 && index < num_obj) {
        return detected_objects[index * NUM_VALUES_PER_OBJECT + distance];
    }
    return -1;
}
std::array<float, 4> Utility::object_box(int index) {
    std::array<float, 4> box;

    if (num_obj == 1) {
        box[0] = detected_objects[x1];
        box[1] = detected_objects[y1];
        box[2] = detected_objects[x2];
        box[3] = detected_objects[y2];
    } else if (index >= 0 && index < num_obj) {
        int startIndex = index * NUM_VALUES_PER_OBJECT;
        box[0] = detected_objects[startIndex + x1];
        box[1] = detected_objects[startIndex + y1];
        box[2] = detected_objects[startIndex + x2];
        box[3] = detected_objects[startIndex + y2];
    }

    return box;
}
void Utility::set_initial_pose(double x, double y, double yaw) {
    // initializationTimer = ros::Time::now();
    odomX = x;
    odomY = y;
    odomYaw = yaw;
    // set_pose_using_service(x, y, yaw);
}
void Utility::update_states_rk4 (double speed, double steering_angle, double dt) {
    if (dt < 0) {
        dt = (ros::Time::now() - timerodom).toSec();
        timerodom = ros::Time::now();
    }
    double magnitude = speed * dt * odomRatio;
    double yaw_rate = magnitude * tan(-steering_angle * M_PI / 180) / wheelbase;

    double k1_x = magnitude * cos(yaw);
    double k1_y = magnitude * sin(yaw);
    double k1_yaw = yaw_rate;

    double k2_x = magnitude * cos(yaw + dt / 2 * k1_yaw);
    double k2_y = magnitude * sin(yaw + dt / 2 * k1_yaw);
    double k2_yaw = yaw_rate;

    double k3_x = magnitude * cos(yaw + dt / 2 * k2_yaw);
    double k3_y = magnitude * sin(yaw + dt / 2 * k2_yaw);
    double k3_yaw = yaw_rate;

    double k4_x = magnitude * cos(yaw + dt * k3_yaw);
    double k4_y = magnitude * sin(yaw + dt * k3_yaw);
    double k4_yaw = yaw_rate;

    dx = 1 / 6.0 * (k1_x + 2 * k2_x + 2 * k3_x + k4_x);
    dy = 1 / 6.0 * (k1_y + 2 * k2_y + 2 * k3_y + k4_y);
    dyaw = 1 / 6.0 * (k1_yaw + 2 * k2_yaw + 2 * k3_yaw + k4_yaw);
    // printf("dt: %.3f, v: %.3f, yaw: %.3f, steer: %.3f, dx: %.3f, dy: %.3f, dyaw: %.3f\n", dt, speed, yaw, steering_angle, dx, dy, dyaw);
}
void Utility::publish_cmd_vel(double steering_angle, double velocity, bool clip) {
    if (velocity < -3.5) velocity = maxspeed;
    if (clip) {
        if (steering_angle > 23) steering_angle = 23;
        if (steering_angle < -23) steering_angle = -23;
    }
    publish_odom();
    lock.lock();
    steer_command = steering_angle;
    velocity_command = velocity;
    lock.unlock();
    float steer = steering_angle;
    float vel = velocity;
    msg.data = "{\"action\":\"1\",\"speed\":" + std::to_string(vel) + "}";
    // ros::Duration(0.01).sleep();
    cmd_vel_pub.publish(msg);
    msg2.data = "{\"action\":\"2\",\"steerAngle\":" + std::to_string(steer) + "}";
    cmd_vel_pub.publish(msg2);
}
void Utility::lane_follow() {
    steer_command = get_steering_angle();
    publish_cmd_vel(steer_command, 0.5);
}
void Utility::idle() {
    steer_command = 0.0;
    velocity_command = 0.0;
    publish_cmd_vel(steer_command, velocity_command);
}
double Utility::get_steering_angle(double offset) {
    ros::Time now = ros::Time::now();
    double dt = 0.1;
    if (timerpid) { 
        dt = (now - timerpid.value()).toSec(); 
        timerpid = now;
    } 
    double error = center - image_center + offset;
    double d_error = (error - last) / dt;
    last = error;
    double steering_angle = (p * error + d * d_error) * 180 / M_PI;
    if (steering_angle > 23) steering_angle = 23;
    if (steering_angle < -23) steering_angle = -23;
    return steering_angle;
}
void Utility::publish_static_transforms() {
    std::vector<geometry_msgs::TransformStamped> static_transforms;

    geometry_msgs::TransformStamped t_camera = add_static_link(0, 0, 0.2, 0, 0, 0, "chassis", "camera");
    static_transforms.push_back(t_camera);

    geometry_msgs::TransformStamped t_imu0 = add_static_link(0, 0, 0, 0, 0, 0, "chassis", "imu0");
    static_transforms.push_back(t_imu0);

    geometry_msgs::TransformStamped t_imu_cam = add_static_link(0, 0, 0.2, 0, 0, 0, "chassis", "imu_cam");
    static_transforms.push_back(t_imu_cam);

    static_broadcaster.sendTransform(static_transforms);
}
geometry_msgs::TransformStamped Utility::add_static_link(double x, double y, double z, double roll, double pitch, double yaw, std::string parent, std::string child) {
    geometry_msgs::TransformStamped t;
    t.header.stamp = ros::Time::now();
    t.header.frame_id = parent;
    t.child_frame_id = child;
    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = z;
    tf2::Quaternion qtn;
    qtn.setRPY(roll, pitch, yaw);
    t.transform.rotation.x = qtn.x();
    t.transform.rotation.y = qtn.y();
    t.transform.rotation.z = qtn.z();
    t.transform.rotation.w = qtn.w();
    return t;
} 
void Utility::callTriggerService() {
    std_srvs::Trigger srv;
    try {
        if (triggerServiceClient.call(srv)) {
            ROS_INFO("Service response: %s", srv.response.message.c_str());
        } else {
            ROS_ERROR("Failed to call service trigger_service");
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Service call failed: %s", e.what());
    }
}
std::array<double, 3> Utility::get_real_states() const {
    return {gps_x, gps_y, yaw};
}
void Utility::spin() {
    while (ros::ok()) {
        ros::spinOnce();
        // rate->sleep();
    }
}

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "utility_node");
//     ros::NodeHandle nh;
//     Utility utility(nh);
//     ros::spin();
//     return 0;
// }
