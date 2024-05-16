#include <ros/ros.h>
#include "utility.hpp"
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
#include <robot_localization/SetPose.h>

Utility::Utility(ros::NodeHandle& nh_, bool real, double x0, double y0, double yaw0, bool subSign, bool useEkf, bool subLane, std::string robot_name, bool subModel, bool subImu, bool pubOdom) 
    : nh(nh_), useIMU(useIMU), subLane(subLane), subSign(subSign), subModel(subModel), subImu(subImu), pubOdom(pubOdom), useEkf(useEkf), robot_name(robot_name),
    trajectoryFunction(nullptr), intersectionDecision(-1), io(), serial(nullptr), real(real)
{
    std::cout << "Utility constructor" << std::endl;
    
    // tunables
    // For odometry
    double sigma_v = 0.1;
    double sigma_delta = 10.0; // degrees
    double odom_publish_frequency = 50; 
    if (real) {
        nh.param<double>("/real/left_trajectory1", left_trajectory1, 3.57);
        nh.param<double>("/real/left_trajectory2", left_trajectory2, -0.86835);
        nh.param<double>("/real/right_trajectory1", right_trajectory1, 3.57);
        nh.param<double>("/real/right_trajectory2", right_trajectory2, -1.17647);
        nh.param<double>("/real/p_rad", p_rad, 3.25);
        nh.param<double>("/real/sigma_v", sigma_v, 0.1);
        nh.param<double>("/real/sigma_delta", sigma_delta, 10.0);
        nh.param<double>("/real/odom_rate", odom_publish_frequency, 50);
    } else {
        nh.param<double>("/sim/left_trajectory1", left_trajectory1, 3.75);
        nh.param<double>("/sim/left_trajectory2", left_trajectory2, -0.49);
        nh.param<double>("/sim/right_trajectory1", right_trajectory1, 3.75);
        nh.param<double>("/sim/right_trajectory2", right_trajectory2, -0.8);
        nh.param<double>("/sim/p_rad", p_rad, 2.35);
        nh.param<double>("/sim/sigma_v", sigma_v, 0.1);
        nh.param<double>("/sim/sigma_delta", sigma_delta, 10.0);
        nh.param<double>("/sim/odom_rate", odom_publish_frequency, 50);
    }
    nh.param<bool>("/hasGps", hasGps, false);
    if (real) {
        serial = std::make_unique<boost::asio::serial_port>(io, "/dev/ttyACM0");
        serial->set_option(boost::asio::serial_port_base::baud_rate(115200));
    }
    q_transform.setRPY(REALSENSE_TF[3], REALSENSE_TF[4], REALSENSE_TF[5]); // 3 values are roll, pitch, yaw of the imu
    // q_transform.setRPY(0, 0.0, 0);
    detected_cars = std::vector<Eigen::Vector2d>();
    detected_cars_counter = std::vector<int>();
    recent_car_indices = std::list<int>();
    nh.getParam("/x_offset", x_offset);
    nh.getParam("/y_offset", y_offset);
    bool model;
    auto ns = ros::this_node::getName();
    if(nh.getParam(ns + "/subModel", model)) {
        this->subModel = model;
    } else {
        std::cout << "failed to get subModel from param server, using default: " << this->subModel << std::endl;
    }
    std::string nodeName = ros::this_node::getName();
    nh.param<double>(nodeName + "/rate", rateVal, 500);
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
    broadcaster = tf2_ros::TransformBroadcaster();
    publish_static_transforms();

    this->x0 = x0;
    this->y0 = y0;
    this->yaw0 = yaw0;
    yaw = yaw0;
    velocity = 0.0;
    odomX = 0.0;
    odomY = 0.0;
    odomX_lidar = 0.0;
    odomY_lidar = 0.0;
    odomYaw = yaw0;
    ekf_x = x0;
    ekf_y = y0;
    ekf_yaw = yaw0;
    gmapping_x = x0;
    gmapping_y = y0;
    if (x0 > 0 && y0 > 0) {
        set_pose_using_service(x0, y0, yaw0);
    }
    initializationFlag = false;
    gps_x = x0;
    gps_y = y0;
    steer_command = 0.0;
    velocity_command = 0.0;
    // car_idx = std::nullopt;

    initializationTimer = std::nullopt;
    timerpid = std::nullopt;

    std::fill(std::begin(odom_msg.pose.covariance), std::end(odom_msg.pose.covariance), 0.0);
    std::fill(std::begin(odom_msg.twist.covariance), std::end(odom_msg.twist.covariance), 0.0);
    std::fill(std::begin(odom_lidar_msg.pose.covariance), std::end(odom_lidar_msg.pose.covariance), 0.0);
    std::fill(std::begin(odom_lidar_msg.twist.covariance), std::end(odom_lidar_msg.twist.covariance), 0.0);
    // covariance_value = 0.01 * 4;
    // for (int hsy=0; hsy<36; hsy+=7) {
    //     odom_msg.pose.covariance[hsy] = covariance_value;
    //     odom_msg.twist.covariance[hsy] = covariance_value;
    //     odom_lidar_msg.pose.covariance[hsy] = covariance_value;
    //     odom_lidar_msg.twist.covariance[hsy] = covariance_value;
    // }
    double dt = 1.0 / odom_publish_frequency;
    double variance_v = sigma_v * sigma_v;
    double sigma_delta_rad = sigma_delta * M_PI / 180;
    double variance_delta = sigma_delta_rad * sigma_delta_rad;
    double variance_yaw_rate = std::pow((sigma_v / 0.27 * std::tan(sigma_delta_rad)), 2);
    double variance_x = variance_v * dt * dt;
    double variance_y = variance_v * dt * dt;
    odom_msg.pose.covariance[0] = variance_x;
    odom_msg.pose.covariance[7] = variance_y;
    odom_msg.pose.covariance[35] = variance_yaw_rate * dt * dt;
    odom_msg.twist.covariance[0] = variance_v;
    odom_msg.twist.covariance[7] = variance_yaw_rate;
    // for (int i = 0; i < 36; i++) {
    //     std::cout << odom_msg.pose.covariance[i] << " ";
    //     if (i % 6 == 5) {
    //         std::cout << std::endl;
    //     }
    // }
    // for (int i = 0; i < 36; i++) {
    //     std::cout << odom_msg.twist.covariance[i] << " ";
    //     if (i % 6 == 5) {
    //         std::cout << std::endl;
    //     }
    // }

    odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 3);
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "chassis";
    // odom1_pub = nh.advertise<nav_msgs::Odometry>("odom1", 3);
    odom_lidar_pub = nh.advertise<nav_msgs::Odometry>("odom1", 3);
    odom_lidar_msg.header.frame_id = "odom";
    odom_lidar_msg.child_frame_id = "chassis";

    // if (robot_name[0] != '/') {
    //     robot_name = "/" + robot_name;
    // }
    cmd_vel_pub = nh.advertise<std_msgs::String>("/" + robot_name + "/command", 3);
    waypoints_pub = nh.advertise<std_msgs::Float32MultiArray>("/waypoints", 3);
    detected_cars_pub = nh.advertise<std_msgs::Float32MultiArray>("/detected_cars", 3);
    state_offset_pub = nh.advertise<std_msgs::Float32MultiArray>("/state_offset", 3);
    
    odom_lidar_sub = nh.subscribe("/odom_lidar", 3, &Utility::odom_lidar_callback, this);
    if (pubOdom) {
        odom_pub_timer = nh.createTimer(ros::Duration(1.0 / odom_publish_frequency), &Utility::odom_pub_timer_callback, this);
    }
    if (useEkf) {
        this->subModel = false;
        ekf_sub = nh.subscribe("/odometry/filtered", 3, &Utility::ekf_callback, this);
    } 
    if (this->subModel) {
        model_sub = nh.subscribe("/gazebo/model_states", 3, &Utility::model_callback, this);
    }
    std::string imu_topic_name;
    bool realsense_imu;
    nh.param<bool>("/realsense_imu", realsense_imu, false);
    std::string car_imu_topic = "/" + robot_name + "/imu";
    if (real) car_imu_topic = "/" + robot_name + "/imu";
    if (realsense_imu) {
        imu_topic_name = "/realsense/imu";
    } else {
        imu_topic_name = car_imu_topic;
    }
    ROS_INFO("imu topic: %s", imu_topic_name.c_str());
    if (subImu) {
        if (!real) {
            std::cout << "waiting for Imu message" << std::endl;
            ros::topic::waitForMessage<sensor_msgs::Imu>(imu_topic_name);
            std::cout << "received message from Imu" << std::endl;
            imu_sub = nh.subscribe(imu_topic_name, 3, &Utility::imu_callback, this);
            // imu_sub = nh.subscribe(imu_topic, 3, &Utility::imu_callback, this);
        } else {
            ROS_INFO("getting imu from serial port");
            // create a ros timer to read from serial port
            imu_pub = nh.advertise<sensor_msgs::Imu>("/car1/imu", 3);
            imu_pub_timer = nh.createTimer(ros::Duration(1.0 / rateVal), &Utility::imu_pub_timer_callback, this);
        }
    }
    if (true) {
        lane_sub = nh.subscribe("/lane", 3, &Utility::lane_callback, this);
        // std::cout << "waiting for lane message" << std::endl;
        // ros::topic::waitForMessage<utils::Lane>("/lane");
        // std::cout << "received message from lane" << std::endl;
        timerpid = ros::Time::now();
    }

    if (subSign) {
        sign_sub = nh.subscribe("/sign", 3, &Utility::sign_callback, this);
        std::cout << "waiting for sign message" << std::endl;
        ros::topic::waitForMessage<std_msgs::Float32MultiArray>("/sign");
        std::cout << "received message from sign" << std::endl;
        car_pose_pub = nh.advertise<std_msgs::Float32MultiArray>("/car_locations", 10);
        car_pose_msg.data.push_back(0.0); // self
        car_pose_msg.data.push_back(0.0);
    }

    nh.param<bool>("/gmapping", useGmapping, false);
    nh.param<bool>("/lidarOdom", useLidarOdom, false);
    nh.param<bool>("/amcl", useAmcl, false);
    if (useGmapping) {
        ROS_INFO("using gmapping");
        tf_sub = nh.subscribe("/tf", 3, &Utility::tf_callback, this);
        pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/chassis_pose", 3);
    } else if(useAmcl) {
        ROS_INFO("using amcl");
        amcl_sub = nh.subscribe("/amcl_pose", 3, &Utility::amcl_callback, this);
        pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/chassis_pose", 3);
    }
    timerodom = ros::Time::now();
}

Utility::~Utility() {
    stop_car(); 
}

void Utility::odom_lidar_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    if(x0 < 0 || y0 < 0) {
        ROS_WARN("odom_lidar_callback: x0 or y0 is less than 0");
        return;
    }
    lock.lock();
    odomX_lidar = msg->pose.pose.position.x;
    odomY_lidar = msg->pose.pose.position.y;
    // ROS_INFO("odomX_lidar: %.3f, odomY_lidar: %.3f", odomX_lidar, odomY_lidar);

    auto current_time = ros::Time::now();
    odom_lidar_msg.header.stamp = current_time;
    odom_lidar_msg.pose.pose.position.x = odomX_lidar + x0;
    odom_lidar_msg.pose.pose.position.y = odomY_lidar + y0;
    odom_lidar_msg.pose.pose.position.z = 0.032939;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
    odom_lidar_msg.pose.pose.orientation = tf2::toMsg(quaternion);
    odom_lidar_pub.publish(odom_lidar_msg);

    lock.unlock();
}
void Utility::tf_callback(const tf2_msgs::TFMessage::ConstPtr& msg) {
    for(const auto& transform : msg->transforms) {
        if (x0 > 0 && y0 > 0 && transform.child_frame_id == "odom" && transform.header.frame_id == "map") {
            double dx = transform.transform.translation.x;
            double dy = transform.transform.translation.y;
            geometry_msgs::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.frame_id = "odom";
            pose_msg.header.stamp = ros::Time::now();
            double odomX1, odomY1;
            if(useLidarOdom) {
                odomX1 = odomX_lidar;
                odomY1 = odomY_lidar;
                ROS_INFO("using lidar odom");
            } else {
                odomX1 = this->odomX;
                odomY1 = this->odomY;
            }
            lock.lock();
            gmapping_x = dx + odomX1 + x0;
            gmapping_y = dy + odomY1 + y0;
            lock.unlock();
            pose_msg.pose.pose.position.x = gmapping_x;
            pose_msg.pose.pose.position.y = gmapping_y;
            pose_msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
            pose_msg.pose.covariance = odom_msg.pose.covariance;
            // ROS_INFO("dx: %.3f, dy: %.3f, x: %.3f, y: %.3f", dx, dy, pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y);
            pose_pub.publish(pose_msg);
        }
    }
}
void Utility::amcl_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    if (x0 < 0 || y0 < 0) {
        ROS_WARN("amcl_callback: x0 or y0 is less than 0");
        return;
    }
    double dx = msg->pose.pose.position.x;
    double dy = msg->pose.pose.position.y;
    // ROS_INFO("amcl_callback: dx: %.3f, dy: %.3f", dx, dy);
    lock.lock();
    gmapping_x = dx + x0;
    gmapping_y = dy + y0;
    lock.unlock();
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.pose.pose.position.x = gmapping_x;
    pose_msg.pose.pose.position.y = gmapping_y;
    pose_msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw));
    pose_msg.pose.covariance = odom_msg.pose.covariance;
    pose_pub.publish(pose_msg);
}
void Utility::odom_pub_timer_callback(const ros::TimerEvent&) {
    publish_odom();
}
void Utility::imu_pub_timer_callback(const ros::TimerEvent&) {
    // auto start = std::chrono::high_resolution_clock::now();
    static char data[256]; // Buffer to store data
    static size_t length = 0;
    static std::string buffer; // Buffer to accumulate the received data
    length = serial->read_some(boost::asio::mutable_buffer(data, 256)); // Read data from serial port
    buffer.append(data, length);
    if (buffer.find("@7") == std::string::npos) {
        // ROS_WARN("cant find @7");
        return;
    }

    // Find the end of line
    size_t end_pos = buffer.find('\n');
    if (end_pos != std::string::npos) {
        std::string line = buffer.substr(0, end_pos); // Extract the line
        buffer.erase(0, end_pos + 1); // Remove the processed part from the buffer

        if (line.find("@7") != std::string::npos) {
            // sensor_msgs::Imu imu_msg;
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "imu0";

            // Extract prefix and ignore '@' and ':' characters
            std::string prefix;
            size_t pos = line.find(':');
            if (pos != std::string::npos && pos + 1 < line.length()) {
                prefix = line.substr(pos + 1);
            } else {
                std::cerr << "Error: Failed to extract prefix from the string." << std::endl;
                return;
            }

            // Extract substrings between ';' characters
            std::string roll_str, pitch_str, yaw_str, accelx_str, accely_str, accelz_str, gyrox_str, gyroy_str, gyroz_str;
            size_t end_pos = pos;
            pos++;
            for (int i = 0; i < 9; ++i) {
                end_pos = line.find(';', pos);
                if (end_pos != std::string::npos) {
                    switch (i) {
                        case 0:
                            roll_str = line.substr(pos, end_pos - pos);
                            break;
                        case 1:
                            pitch_str = line.substr(pos, end_pos - pos);
                            break;
                        case 2:
                            yaw_str = line.substr(pos, end_pos - pos);
                            break;
                        case 3:
                            accelx_str = line.substr(pos, end_pos - pos);
                            break;
                        case 4:
                            accely_str = line.substr(pos, end_pos - pos);
                            break;
                        case 5:
                            accelz_str = line.substr(pos, end_pos - pos);
                            break;
                        case 6:
                            gyrox_str = line.substr(pos, end_pos - pos);
                            break;
                        case 7:
                            gyroy_str = line.substr(pos, end_pos - pos);
                            break;
                        case 8:
                            gyroz_str = line.substr(pos, end_pos - pos);
                            break;
                    }
                    pos = end_pos + 1;
                } else {
                    std::cerr << "Error: Failed to find delimiter in the string." << std::endl;
                    return;
                }
            }

            // Convert substrings to floating-point numbers
            static double roll;
            static double pitch;
            static double yaw_deg;
            static double accelx;
            static double accely;
            static double accelz;
            static double gyrox;
            static double gyroy;
            static double gyroz;
            try {
                roll = stod(roll_str);
                pitch = stod(pitch_str);
                yaw_deg = stod(yaw_str);
                accelx = stod(accely_str);
                accely = stod(accelx_str);
                accelz = stod(accelz_str);
                gyrox = stod(gyrox_str);
                gyroy = stod(gyroy_str);
                gyroz = stod(gyroz_str);
            } catch (const std::invalid_argument& e) {
                std::cerr << "Error: Failed to convert string to floating-point number." << std::endl;
                return;
            }

            static double accel_mag;


            lock.lock();
            this->yaw = -yaw_deg * M_PI/180;
            while(this->yaw < -M_PI) this->yaw += 2*M_PI;
            while(this->yaw > M_PI) this->yaw -= 2*M_PI;
            lock.unlock();

            static bool debug_imu = false;
            if (debug_imu) {
                printf("roll: %.2f, pitch: %.2f, yaw: %.2f, accelx: %.2f, accely: %.2f, accelz: %.2f, gyrox: %.2f, gyroy: %.2f, gyroz: %.2f\n", roll, pitch, yaw_deg, accelx, accely, accelz, gyrox, gyroy, gyroz);
            }

            // Convert Euler angles to quaternion
            tf2::Quaternion q;
            q.setRPY(roll*M_PI/180, pitch*M_PI/180, yaw_deg*M_PI/180);
            imu_msg.orientation.x = q.x();
            imu_msg.orientation.y = q.y();
            imu_msg.orientation.z = q.z();
            imu_msg.orientation.w = q.w();

            // Set linear acceleration
            imu_msg.linear_acceleration.x = accelx;
            imu_msg.linear_acceleration.y = accely;
            imu_msg.linear_acceleration.z = accelz;

            imu_msg.angular_velocity.x = gyrox*2*M_PI;
            imu_msg.angular_velocity.y = gyroy*2*M_PI;
            imu_msg.angular_velocity.z = gyroz*2*M_PI;

            imu_pub.publish(imu_msg); // Publish the IMU message
            // auto end = std::chrono::high_resolution_clock::now();
            // std::chrono::duration<double> elapsed = end - start;
            // ROS_INFO("imu_pub_timer_callback time elapsed: %fs, rate = %fhz", elapsed.count(), 1/elapsed.count());
        } 
        // else {
        //     ROS_INFO("line.find(@7) == std::string::npos");
        // }
    } 
    // else {
    //     ROS_WARN("end_pos == std::string::npos");
    // }
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
    static bool populate_car_pose = true;
    if (!populate_car_pose) {
        return;
    }
    int car_id = 12;
    double threshold = 0.5;
    for(int i = 0; i < num_obj; i++) {
        if(msg->data[i * NUM_VALUES_PER_OBJECT + id] == car_id) {
            double dist = object_distance(i);
            if(dist > 3.0 || dist < 0.6) continue;
            double xmin = msg->data[i * NUM_VALUES_PER_OBJECT + VehicleConstants::x1];
            double ymin = msg->data[i * NUM_VALUES_PER_OBJECT + VehicleConstants::y1];
            double xmax = msg->data[i * NUM_VALUES_PER_OBJECT + VehicleConstants::x2];
            double ymax = msg->data[i * NUM_VALUES_PER_OBJECT + VehicleConstants::y2];
            double x, y, yaw;
            get_states(x, y, yaw);
            Eigen::Vector2d world_pose = estimate_object_pose2d(x, y, yaw, xmin, ymin, xmax, ymax, dist, CAMERA_PARAMS, true);
            // std::cout << "world_pose: (" << world_pose[0] << ", " << world_pose[1] << "), self pose: (" << x << ", " << y << ")" << std::endl;
            // check error norm between x, y of detected car and the ones in the detected_cars vector
            // if error norm greater than threshold, add to detected_cars vector
            // else, update the x, y of the detected car in the detected_cars vector by averaging 
            if (detected_cars.size() == 0) {
                detected_cars.push_back(world_pose);
                detected_cars_counter.push_back(1);
                car_pose_msg.data.push_back(world_pose[0]);
                car_pose_msg.data.push_back(world_pose[1]);
                // std::cout << "new car detected at (" << world_pose[0] << ", " << world_pose[1] << ")" << std::endl;
                continue;
            }
            for(int j = 0; j < detected_cars.size(); j++) {
                double error_norm_sq = (detected_cars[j] - world_pose).squaredNorm();
                double score = msg->data[i * NUM_VALUES_PER_OBJECT + confidence];
                if(error_norm_sq < threshold * threshold) {
                    // if (detected_cars_counter[j] < 15) {
                    if (true) {
                        // detected_cars[j] = (detected_cars[j] * detected_cars_counter[j] + world_pose) / (detected_cars_counter[j] + 1);
                        detected_cars[j] = (detected_cars[j] * 0.1 + world_pose * 0.9);
                        car_pose_msg.data[(j+1) * 2] = detected_cars[j][0];
                        car_pose_msg.data[(j+1) * 2 + 1] = detected_cars[j][1];
                        // std::cout << "updated car detected at (" << detected_cars[j][0] << ", " << detected_cars[j][1] << ")" << std::endl;
                        detected_cars_counter[j]++;
                    }
                    recent_car_indices.remove(j); // remove j if it exists
                    recent_car_indices.push_front(j); // add j to the front
                    break;
                } else if(j == detected_cars.size() - 1) {
                    detected_cars.push_back(world_pose);
                    car_pose_msg.data.push_back(world_pose[0]);
                    car_pose_msg.data.push_back(world_pose[1]);
                    detected_cars_counter.push_back(1);
                    recent_car_indices.push_front(detected_cars.size() - 1); // Add new car as most recent
                    // std::cout << "new car detected at (" << world_pose[0] << ", " << world_pose[1] << "), num cars: " << detected_cars.size() << std::endl;
                    break;
                }
            }
            while (recent_car_indices.size() > 5) {
                recent_car_indices.pop_back(); // keep only 4 most recent cars
            }
            static bool publish_cars = true;
            if(publish_cars) {
                std_msgs::Float32MultiArray car_msg;
                car_msg.data = car_pose_msg.data;
                car_pose_pub.publish(car_msg);
            }
        }
    }
    // print car_pose_msg
    // for (int i = 0; i < car_pose_msg.data.size(); i += 2) {
    //     std::cout << "car " << i / 2 << ": (" << car_pose_msg.data[i] << ", " << car_pose_msg.data[i + 1] << ")" << std::endl;
    // }
    car_pose_pub.publish(car_pose_msg);
}
void Utility::lane_callback(const utils::Lane::ConstPtr& msg) {
    static double previous_center = 320;
    lock.lock();
    center = msg->center;
    if(std::abs(center - previous_center) > 200) {
        // ROS_INFO("center is too far from previous_center");
        center = previous_center;
    }
    if (std::abs(center - 320) < 0.01) {
        // ROS_INFO("center is 320");
        double temp = center;
        center = previous_center;
        previous_center = temp;
    } else {
        previous_center = center;
    }
    stopline = msg->stopline;
    lock.unlock();
}
void Utility::imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    lock.lock();
    // this->imu_msg = *msg;
    q_imu = tf2::Quaternion(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    // q_chassis = q_transform * q_imu;
    // q_chassis.normalize();
    // m_chassis = tf2::Matrix3x3(q_chassis);
    double roll, pitch;

    m_chassis = tf2::Matrix3x3(q_imu); // No transformation

    m_chassis.getRPY(roll, pitch, yaw);

    if (real) yaw *= -1;
    // ROS_INFO("yaw: %.3f", yaw * 180 / M_PI);
    // ROS_INFO("yaw: %.3f, angular velocity: %.3f, acceleration: %.3f, %.3f, %.3f", yaw * 180 / M_PI, msg->angular_velocity.z, msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    if (!imuInitialized) {
        imuInitialized = true;
        if (real) initial_yaw = yaw;
        std::cout << "imu initialized" << ", intial yaw is " << yaw * 180 / M_PI << " degrees" << std::endl;
    }
    if (!initializationFlag && x0 > 0 && y0 > 0) {
        initializationFlag = true;
        ROS_INFO("initialized in imu callback");
    }
    // yaw = yaw - initial_yaw + yaw0;
    while(yaw < -M_PI) {
        yaw += 2*M_PI;
    }
    while(yaw > M_PI) {
        yaw -= 2*M_PI;
    }
    lock.unlock();
}
void Utility::ekf_callback(const nav_msgs::Odometry::ConstPtr& msg) {
    // ros::Time now = ros::Time::now();
    lock.lock();
    ekf_x = msg->pose.pose.position.x;
    ekf_y = msg->pose.pose.position.y;
    // tf2::fromMsg(msg->pose.pose.orientation, tf2_quat);
    // ekf_yaw = tf2::impl::getYaw(tf2_quat);
    if (subSign) {
        car_pose_msg.data[0] = ekf_x;
        car_pose_msg.data[1] = ekf_y;
    }
    // if (useEkf) {
    //     if (!initializationFlag) {
    //         if (x0 < 0 || y0 < 0) {
    //             std::cout << "using ekf but haven't received data yet. ekf_x: " << ekf_x << ", ekf_y: " << ekf_y << std::endl;
    //             x0 = ekf_x;
    //             y0 = ekf_y;
    //             gmapping_x = ekf_x;
    //             gmapping_y = ekf_y;
    //         } else {
    //             ROS_INFO("Initializing... ekf_x: %.3f, ekf_y: %.3f", ekf_x, ekf_y);
    //         }
    //         if (imuInitialized) initializationFlag = true;
    //     }
    // }
    lock.unlock();
    // ROS_INFO("ekf callback rate: %f", 1 / (now - general_timer).toSec());
    // general_timer = now;
}
void Utility::model_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // static ros::Time last_time;
    // ros::Time now = ros::Time::now();
    // double dt = (now - last_time).toSec();
    // last_time = now;
    // ROS_INFO("model callback rate: %.3f", 1 / dt);

    lock.lock();
    // auto start = std::chrono::high_resolution_clock::now();
    if (!car_idx.has_value()) {
        auto it = std::find(msg->name.begin(), msg->name.end(), robot_name);
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
    if (subSign && !useEkf) {
        car_pose_msg.data[0] = gps_x;
        car_pose_msg.data[1] = gps_y;
    }
    lock.unlock();
}

void Utility::stop_car() {
    std::cout << "Stopping car" << std::endl;
    msg.data = "{\"action\":\"1\",\"speed\":" + std::to_string(0.0) + "}";
    msg2.data = "{\"action\":\"2\",\"steerAngle\":" + std::to_string(0.0) + "}";
    for (int i = 0; i < 10; i++) {
        publish_cmd_vel(0.0, 0.0);
        ros::Duration(0.15).sleep();
    }
    std::cout << "sent commands to stop car" << std::endl;
}

void Utility::set_pose_using_service(double x, double y, double yaw) {
    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = "odom";
    pose_msg.pose.pose.position.x = x;
    pose_msg.pose.pose.position.y = y;
    Eigen::Quaterniond q(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    pose_msg.pose.pose.orientation.x = q.x();
    pose_msg.pose.pose.orientation.y = q.y();
    pose_msg.pose.pose.orientation.z = q.z();
    pose_msg.pose.pose.orientation.w = q.w();

    if (useEkf) {
        std::cout << "waiting for set_pose service" << std::endl;
        ros::service::waitForService("/set_pose");
    }

    try {
        ros::ServiceClient client = nh.serviceClient<robot_localization::SetPose>("/set_pose");
        robot_localization::SetPose srv;
        srv.request.pose = pose_msg;
        if (client.call(srv)) {
            ROS_INFO("Service call successful");
        } else {
            ROS_ERROR("Failed to call service set_pose");
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Service call failed: %s", e.what());
    }
}

void Utility::publish_odom() {
    yaw = fmod(yaw, 2 * M_PI);

    // update_states_rk4(velocity, steer_command);
    update_states_rk4(velocity_command, steer_command);

    odomX += dx;
    odomY += dy;
    // ROS_INFO("odomX: %.3f, gps_x: %.3f, odomY: %.3f, gps_y: %.3f, error: %.3f", odomX, gps_x, odomY, gps_y, sqrt((odomX - gps_x) * (odomX - gps_x) + (odomY - gps_y) * (odomY - gps_y))); // works
    
    auto current_time = ros::Time::now();
    odom_msg.header.stamp = current_time;
    odom_msg.pose.pose.position.x = odomX;
    odom_msg.pose.pose.position.y = odomY;
    odom_msg.pose.pose.position.z = 0.032939;

    // odom_msg.twist.twist.linear.x = x_speed;
    // odom_msg.twist.twist.linear.y = y_speed;
    // odom_msg.twist.twist.angular.z = this->imu_msg.angular_velocity.z;
    odom_msg.twist.twist.linear.x = velocity_command; // non-holonomic, x means forward
    odom_msg.twist.twist.linear.y = 0; // y means lateral
    odom_msg.twist.twist.angular.z = this->imu_msg.angular_velocity.z;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
    odom_msg.pose.pose.orientation = tf2::toMsg(quaternion);

    odom_pub.publish(odom_msg);

    std_msgs::Float32MultiArray state_offset_msg;
    state_offset_msg.data.push_back(x0);
    state_offset_msg.data.push_back(y0);
    state_offset_pub.publish(state_offset_msg);

    static bool publish_tf = !useLidarOdom;
    if (publish_tf) {
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = current_time;
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = "chassis";
        transformStamped.transform.translation.x = odomX;
        transformStamped.transform.translation.y = odomY;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        transformStamped.transform.rotation = tf2::toMsg(q);
        broadcaster.sendTransform(transformStamped);
    }
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

std::vector<int> Utility::object_indices(int obj_id) {
    std::vector<int> indices;
    if (num_obj < 1) {
        return indices;
    }
    if (num_obj == 1) {
        if (detected_objects[id] == obj_id) {
            indices.push_back(0);
        }
        return indices;
    }
    for (int i = 0; i < num_obj; ++i) {
        if (detected_objects[i * NUM_VALUES_PER_OBJECT + id] == obj_id) {
            indices.push_back(i);
        }
    }
    return indices;
}

double Utility::object_distance(int index) {
    if (num_obj == 1) {
        return detected_objects[distance];
    } else if (index >= 0 && index < num_obj) {
        return detected_objects[index * NUM_VALUES_PER_OBJECT + distance];
    }
    return -1;
}
std::array<double, 4> Utility::object_box(int index) {
    std::array<double, 4> box;

    if (num_obj == 1) {
        box[0] = detected_objects[VehicleConstants::x1];
        box[1] = detected_objects[VehicleConstants::y1];
        box[2] = detected_objects[VehicleConstants::x2];
        box[3] = detected_objects[VehicleConstants::y2];
    } else if (index >= 0 && index < num_obj) {
        int startIndex = index * NUM_VALUES_PER_OBJECT;
        box[0] = detected_objects[startIndex + VehicleConstants::x1];
        box[1] = detected_objects[startIndex + VehicleConstants::y1];
        box[2] = detected_objects[startIndex + VehicleConstants::x2];
        box[3] = detected_objects[startIndex + VehicleConstants::y2];
    }

    return box;
}
void Utility::object_box(int index, std::array<double, 4>& oBox) {
    if (num_obj == 1) {
        oBox[0] = detected_objects[VehicleConstants::x1];
        oBox[1] = detected_objects[VehicleConstants::y1];
        oBox[2] = detected_objects[VehicleConstants::x2];
        oBox[3] = detected_objects[VehicleConstants::y2];
    } else if (index >= 0 && index < num_obj) {
        int startIndex = index * NUM_VALUES_PER_OBJECT;
        oBox[0] = detected_objects[startIndex + VehicleConstants::x1];
        oBox[1] = detected_objects[startIndex + VehicleConstants::y1];
        oBox[2] = detected_objects[startIndex + VehicleConstants::x2];
        oBox[3] = detected_objects[startIndex + VehicleConstants::y2];
    }
}
void Utility::set_initial_pose(double x, double y, double yaw) {
    // initializationTimer = ros::Time::now();
    ROS_INFO("Setting initial pose: x: %.3f, y: %.3f, yaw: %.3f", x, y, yaw);
    odomX = x;
    odomY = y;
    odomYaw = yaw;
    // set_pose_using_service(x, y, yaw);
}
void Utility::reset_odom() {
    set_initial_pose(0, 0, 0);
}
int Utility::update_states_rk4 (double speed, double steering_angle, double dt) {
    if (dt < 0) {
        dt = (ros::Time::now() - timerodom).toSec();
        timerodom = ros::Time::now();
    }
    if (dt > (0.1)) {
        ROS_WARN("update_states_rk4(): dt is too large: %.3f", dt);
        return 0;
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
    return 1;
}
void Utility::publish_cmd_vel(double steering_angle, double velocity, bool clip) {
    if (velocity < -3.5) velocity = maxspeed;
    if (clip) {
        if (steering_angle > HARD_MAX_STEERING) steering_angle = HARD_MAX_STEERING;
        if (steering_angle < -HARD_MAX_STEERING) steering_angle = -HARD_MAX_STEERING;
    }
    // Check for NaN values and handle them
    if (std::isnan(steering_angle)) {
        std::cerr << "Error: Steering angle is NaN!" << std::endl;
        steering_angle = 0.0;
    }

    if (std::isnan(velocity)) {
        std::cerr << "Error: Velocity is NaN!" << std::endl;
        velocity = 0.0;
    }
    float vel = velocity;
    lock.lock();
    steer_command = steering_angle;
    velocity_command = velocity;

    lock.unlock();
    float steer = steering_angle;
    if(real) {
        send_speed_and_steer(vel, steer);
        // ROS_INFO("publishing steer: %.3f, speed: %.3f", steer, vel);
        msg2.data = "{\"action\":\"2\",\"steerAngle\":" + std::to_string(steer) + "}";
        cmd_vel_pub.publish(msg2);
        // ros::Duration(0.03).sleep();
        msg.data = "{\"action\":\"1\",\"speed\":" + std::to_string(vel) + "}";
        cmd_vel_pub.publish(msg);
    } else {
        msg2.data = "{\"action\":\"2\",\"steerAngle\":" + std::to_string(steer) + "}";
        cmd_vel_pub.publish(msg2);
        // ros::Duration(0.03).sleep();
        msg.data = "{\"action\":\"1\",\"speed\":" + std::to_string(vel) + "}";
        cmd_vel_pub.publish(msg);
    }
}
void Utility::lane_follow() {
    steer_command = get_steering_angle();
    publish_cmd_vel(steer_command, 0.175);
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
    if (steering_angle > HARD_MAX_STEERING) steering_angle = HARD_MAX_STEERING;
    if (steering_angle < -HARD_MAX_STEERING) steering_angle = -HARD_MAX_STEERING;
    return steering_angle;
}
void Utility::publish_static_transforms() {
    std::vector<geometry_msgs::TransformStamped> static_transforms;

    geometry_msgs::TransformStamped t_camera = add_static_link(0, 0, 0.2, 0, 0, 0, "chassis", "camera");
    static_transforms.push_back(t_camera);

    geometry_msgs::TransformStamped t_laser = add_static_link(0, 0, 0.2, 0, 0, M_PI/2, "chassis", "laser");
    static_transforms.push_back(t_laser);

    geometry_msgs::TransformStamped t_imu0 = add_static_link(0, 0, 0, 0, 0, 0, "chassis", "imu0");
    static_transforms.push_back(t_imu0);

    geometry_msgs::TransformStamped t_imu_cam = add_static_link(0.1, 0, 0.16, 0, 0.15, 0, "chassis", "realsense");
    static_transforms.push_back(t_imu_cam);

    geometry_msgs::TransformStamped t_imu_cam2 = add_static_link(0.1, 0, 0.16, 0, 0.15, 0, "chassis", "camera_imu_optical_frame");
    static_transforms.push_back(t_imu_cam2);

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
    // auto start = std::chrono::high_resolution_clock::now();
    ROS_INFO("Utility node spinning at a rate of %f", rateVal);
    while (ros::ok()) {
        // auto end = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double> elapsed = end - start;
        // start = end;
        // ROS_INFO("utility_node loop time elapsed: %fs, rate: %f", elapsed.count(), 1 / elapsed.count());
        ros::spinOnce();
        rate->sleep();
    }
}

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "utility_node");
//     ros::NodeHandle nh;
//     Utility utility(nh);
//     ros::spin();
//     return 0;
// }
