#include <ros/ros.h>
#include <thread>
#include <map>
#include <string>
#include <vector>
#include <mutex>
#include "utility.hpp"
#include "optimizer.hpp"
#include <signal.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include "utils/waypoints.h"
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <ncurses.h>

using namespace VehicleConstants;

class StateMachine {
public:
    StateMachine(ros::NodeHandle& nh_, double T, int N, double v_ref, bool sign, bool ekf, bool lane, double T_park, std::string robot_name, double x_init, double y_init, double yaw_init, bool real): 
    nh(nh_), utils(nh, real, x_init, y_init, yaw_init, sign, ekf, lane, robot_name), mpc(T,N,v_ref), xs(5),
    state(STATE::INIT), sign(sign), ekf(ekf), lane(lane), T_park(T_park), T(T), end_index(0), real(real)
    {
        // tunables
        if(real) {
            nh.param<double>("/real/straight_trajectory_threshold", straight_trajectory_threshold, 1.3);
            nh.param<double>("/real/right_trajectory_threshold", right_trajectory_threshold, 5.73 * M_PI/180);
            nh.param<double>("/real/left_trajectory_threshold", left_trajectory_threshold, 5.73 * M_PI/180);
            
            nh.param<double>("/real/change_lane_yaw", change_lane_yaw, 0.15);
            nh.param<double>("/real/cw_speed_ratio", cw_speed_ratio, 0.7143);
            nh.param<double>("/real/hw_speed_ratio", hw_speed_ratio, 1.33);
            nh.param<double>("/real/sign_localization_threshold", sign_localization_threshold, 0.5);
            nh.param<double>("/real/lane_localization_orientation_threshold", lane_localization_orientation_threshold, 10);
            nh.param<double>("/real/pixel_center_offset", pixel_center_offset, 0.0);
            nh.param<double>("/real/constant_distance_to_intersection_at_detection", constant_distance_to_intersection_at_detection, 0.371);
            nh.param<double>("/real/intersection_localization_threshold", intersection_localization_threshold, 0.5);
        } else {
            nh.param<double>("/sim/straight_trajectory_threshold", straight_trajectory_threshold, 1.3);
            nh.param<double>("/sim/right_trajectory_threshold", right_trajectory_threshold, 5.73 * M_PI/180);
            nh.param<double>("/sim/left_trajectory_threshold", left_trajectory_threshold, 5.73 * M_PI/180);
            nh.param<double>("/sim/change_lane_yaw", change_lane_yaw, 0.25);
            nh.param<double>("/sim/cw_speed_ratio", cw_speed_ratio, 0.7143);
            nh.param<double>("/sim/hw_speed_ratio", hw_speed_ratio, 1.33);
            nh.param<double>("/sim/sign_localization_threshold", sign_localization_threshold, 0.5);
            nh.param<double>("/sim/lane_localization_orientation_threshold", lane_localization_orientation_threshold, 10);
            nh.param<double>("/sim/pixel_center_offset", pixel_center_offset, -30.0);
            nh.param<double>("/sim/constant_distance_to_intersection_at_detection", constant_distance_to_intersection_at_detection, 0.371);
            nh.param<double>("/sim/intersection_localization_threshold", intersection_localization_threshold, 0.5);
        }
        left_trajectory_threshold *= M_PI/180;
        right_trajectory_threshold *= M_PI/180;

        nh.param("pub_wpts", pubWaypoints, true);
        nh.param("/kb", keyboardControl, false);
        if (keyboardControl) {
            std::cout << "keyboard control enabled" << std::endl;
            change_state(STATE::KEYBOARD_CONTROL);
        }
        if(!nh.getParam("/pathName", pathName)) {
            ROS_ERROR("Failed to get param 'pathName'");
            pathName = "path1";
        }
        nh.param("/dashboard", dashboard, true);
        nh.param("/gps", hasGps, false);

        //initialize parking spots
        for(int i=0; i<5; i++) {
            Eigen::Vector2d spot_right = {PARKING_SPOT_RIGHT[0] + i*PARKING_SPOT_LENGTH, PARKING_SPOT_RIGHT[1]};
            Eigen::Vector2d spot_left = {PARKING_SPOT_LEFT[0] + i*PARKING_SPOT_LENGTH, PARKING_SPOT_LEFT[1]};
            PARKING_SPOTS.push_back(spot_right);
            PARKING_SPOTS.push_back(spot_left);
        }

        double rateVal = 1/mpc.T;
        rate = new ros::Rate(rateVal);
        std::cout << "rate: " << rateVal << std::endl;
        start_trigger = nh.advertiseService("/start_bool", &StateMachine::start_bool_callback, this);
        ROS_INFO("server ready, mpc time step T = %.3f", T);
        std::cout << "state machine initialized" << std::endl;
    }
    ~StateMachine() {
        // utils.stop_car();
    }
    ros::NodeHandle& nh;

// private:
    //tunables
    double straight_trajectory_threshold = 1.3, right_trajectory_threshold = 5.73 * M_PI/180, left_trajectory_threshold = 5.73 * M_PI/180, 
            change_lane_yaw = 0.15, cw_speed_ratio, hw_speed_ratio, sign_localization_threshold = 0.5, 
            lane_localization_orientation_threshold = 10, pixel_center_offset = -30.0, constant_distance_to_intersection_at_detection = 0.371,
            intersection_localization_threshold = 0.5;

    std::vector<Eigen::Vector2d> PARKING_SPOTS;

    // std::array<int, 9> maneuver_indices = {2, 2, 2, 2, 2, 2, 2, 2, 2};
    std::vector<int> maneuver_indices = {1,0,2,0,1,2,0,0,0};
    int maneuver_index = 0;
    std::array<double, 4> bbox = {0.0, 0.0, 0.0, 0.0};
    double T_park, T;
    double detected_dist = 0;
    bool right_park = true;
    int park_count = 0;
    int stopsign_flag = 0; // stopsign, traffic light, priority, roundabout
    int maneuver_direction = 0; // 0: left, 1: straight, 2: right
    ros::Time cd_timer = ros::Time::now();
    int end_index = 0;
    Eigen::Vector2d destination;
    Eigen::VectorXd xs;
    int state = 0;
    bool debug = true, sign, ekf, lane, real, dashboard, keyboardControl, hasGps, pubWaypoints;
    std::string pathName;
    double running_x, running_y, running_yaw;

    std::array<double, 3> x0;
    ros::Rate* rate;

    std::mutex lock;
    Utility utils;
    Optimizer mpc;

    void call_trigger_service() {
        ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/trigger_service");
        std_srvs::Trigger srv;
        client.call(srv);
    }
    int start() {
        // sleep 2 seconds to allow for initialization
        ros::Duration(2).sleep();
        if(hasGps) {
            if(!utils.reinitialize_states()) ROS_WARN("Failed to reinitialize");
        }
        double x, y, yaw;
        utils.get_states(x, y, yaw);
        ROS_INFO("start(): x=%.3f, y=%.3f, yaw=%.3f", x, y, yaw);
        ros::ServiceClient waypoints_client = nh.serviceClient<utils::waypoints>("/waypoint_path");
        utils::waypoints srv;
        srv.request.pathName = pathName;
        srv.request.x0 = x;
        srv.request.y0 = y;
        srv.request.yaw0 = yaw;
        //convert v_ref to string
        int vrefInt;
        if(!nh.getParam("/vrefInt", vrefInt)) {
            ROS_ERROR("Failed to get param 'vrefInt'");
            vrefInt = 25;
        }
        srv.request.vrefName = std::to_string(vrefInt);
        ROS_INFO("waiting for waypoints service");
        if(waypoints_client.waitForExistence(ros::Duration(5))) {
            ROS_INFO("waypoints service found");
        } else {
            ROS_ERROR("waypoints service not found after 5 seconds");
        }
        if(waypoints_client.call(srv)) {
            std::vector<double> state_refs(srv.response.state_refs.data.begin(), srv.response.state_refs.data.end()); // N by 3
            std::vector<double> input_refs(srv.response.input_refs.data.begin(), srv.response.input_refs.data.end()); // N by 2
            std::vector<double> wp_attributes(srv.response.wp_attributes.data.begin(), srv.response.wp_attributes.data.end()); // N by 1
            std::vector<double> wp_normals(srv.response.wp_normals.data.begin(), srv.response.wp_normals.data.end()); // N by 2
            std::vector<int> maneuver_directions(srv.response.maneuver_directions.data.begin(), srv.response.maneuver_directions.data.end()); // N by 1
            maneuver_indices = maneuver_directions;
            for (int i = 0; i < maneuver_indices.size(); i++) {
                std::cout << "maneuver direction " << i << ": " << maneuver_indices[i] << std::endl;
            }
            // exit(0);
            int N = state_refs.size() / 3;
            mpc.state_refs = Eigen::Map<Eigen::MatrixXd>(state_refs.data(), 3, N).transpose();
            mpc.input_refs = Eigen::Map<Eigen::MatrixXd>(input_refs.data(), 2, N).transpose();
            mpc.state_attributes = Eigen::Map<Eigen::VectorXd>(wp_attributes.data(), N);
            mpc.normals = Eigen::Map<Eigen::MatrixXd>(wp_normals.data(), 2, N).transpose();
            // for (int i = 0; i < N/5; i++) {
            //     std::cout << "state refs " << i << ": " << mpc.state_refs.row(i) << std::endl;
            // }
            // exit(0);
            ROS_INFO("Received waypoints of size %d", N);
        } else {
            ROS_ERROR("Failed to call service waypoints");
        }
        mpc.initialize_current_states(x, y, utils.yaw);
        destination = mpc.state_refs.row(mpc.state_refs.rows()-1).head(2);
        ROS_INFO("destination: %.3f, %.3f", destination(0), destination(1));

        mpc.target_waypoint_index = mpc.find_closest_waypoint(0, mpc.state_refs.rows()-1); // search from the beginning to the end
        mpc.reset_solver();
        if (lane) {
            change_state(STATE::LANE_FOLLOWING);
        } else {
            change_state(STATE::MOVING);
        }
        return 1;
    }
    bool start_bool_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
        static int history = -1;
        if (req.data && state == STATE::INIT) {
            if (history == -1) {
                if (lane) {
                    change_state(STATE::LANE_FOLLOWING);
                } else {
                    change_state(STATE::MOVING);
                }
            } else {
                change_state(static_cast<STATE>(history));
            }
            res.success = true;
            res.message = "Started";
        } else {
            history = state;
            stop_for(10*T);
            change_state(STATE::INIT);
            res.success = true;
            res.message = "Stopped";
        }
        return true;
        
    }
    ros::ServiceServer start_trigger;
    void solve(bool safety_check = true);
    void publish_commands();
    void update_mpc_state();
    void change_state(STATE new_state);
    void run();
    void stop_for(double duration) {
        ros::Time timer = ros::Time::now() + ros::Duration(duration);
        while (ros::Time::now() < timer) {
            utils.publish_cmd_vel(0.0, 0.0);
            rate->sleep();
        }
    }
    int parking_maneuver_hardcode(bool right=true, bool exit=false, double rate_val=20, double initial_y_error = 0, double initial_yaw_error = 0) {
        Eigen::VectorXd targets(3);
        Eigen::VectorXd steerings(3);
        Eigen::VectorXd speeds(3);
        Eigen::VectorXd thresholds(3);
        double base_yaw_target = 0.166 * M_PI;
        if (!real) base_yaw_target = 0.29 * M_PI;
        base_yaw_target = base_yaw_target + 0.02 / (0.29 * M_PI) * base_yaw_target * initial_y_error / MAX_PARKING_Y_ERROR * (right ? 1 : -1);
        ROS_INFO("initial y error: %.3f, initial yaw error: %.3f, base yaw target: %.3f PI", initial_y_error, initial_yaw_error, base_yaw_target / M_PI);
        // if (!real) base_yaw_target = 0.31 * M_PI;
        // if (!real) base_yaw_target = 0.27 * M_PI;
        double base_steer = - HARD_MAX_STEERING;
        // base_steer = - 20;
        double base_speed = -0.2;
        double base_thresh = 0.1;
        targets << base_yaw_target, 0.0, 0.0;
        steerings << -base_steer, base_steer, -base_steer;
        speeds << base_speed, base_speed, -base_speed;
        thresholds << base_thresh, base_thresh, base_thresh/3;
        if (exit) {
            base_yaw_target *= 0.95;
            targets << base_yaw_target, base_yaw_target, 0.0;
            thresholds(0) = (1 - base_thresh / base_yaw_target) * base_yaw_target;
            thresholds(1) = base_thresh;
            thresholds(2) = base_thresh;
            speeds << base_speed, -base_speed, -base_speed;
        }
        if (!right) {
            steerings *= -1;
            targets *= -1;
            std::cout << "targets: " << targets.transpose() << ", steerings: " << steerings.transpose() << ", speeds: " << speeds.transpose() << ", thresholds: " << thresholds.transpose() << std::endl;
        }
        ROS_INFO("parking maneuver called. park right: %s, exit: %s", right ? "true" : "false", exit ? "true" : "false");
        std::cout << "targets: " << targets.transpose() << ", steerings: " << steerings.transpose() << ", speeds: " << speeds.transpose() << ", thresholds: " << thresholds.transpose() << std::endl;
        return maneuver_hardcode(targets, steerings, speeds, thresholds, rate_val);
    }
    int change_lane_hardcode(bool right, double yaw_offset = 0, double rate_val = 20) {
        Eigen::VectorXd targets(1);
        double yaw_target = change_lane_yaw * M_PI;
        Eigen::VectorXd steering(1);
        steering << -23.;
        if (right) {
            steering *= -1;
            yaw_target *= -1;
        }
        yaw_target += yaw_offset;
        targets << yaw_target;
        Eigen::VectorXd speeds(1);
        speeds << 0.2;
        Eigen::VectorXd thresholds(1);
        thresholds << 0.05;
        return maneuver_hardcode(targets, steering, speeds, thresholds, rate_val);
    }
    int maneuver_hardcode(const Eigen::VectorXd &targets, const Eigen::VectorXd &steerings, const Eigen::VectorXd &speeds, const Eigen::VectorXd &thresholds, double rate_val = 20) {
        /* 
        targets: target yaws or distances
        thresholds: thresholds for each target        
        */
        double x0, y0, yaw0;
        // get current states
        utils.get_states(x0, y0, yaw0);
        // get closest direction
        yaw0 = mpc.NearestDirection(yaw0);
        while (yaw0 < -M_PI) yaw0 += 2*M_PI;
        while (yaw0 > M_PI) yaw0 -= 2*M_PI;
        Eigen::VectorXd yaw0_vec = Eigen::VectorXd::Constant(targets.size(), yaw0);
        Eigen::VectorXd target_yaws = targets + yaw0_vec;
        std::cout << "target yaws: " << target_yaws.transpose() << std::endl;
        ROS_INFO("maneuver called. nearest direction: %.3f", yaw0);
        int stage = 1;
        int num_stages = targets.size();
        ros::Rate temp_rate(rate_val);
        
        double steering_angle = steerings(stage-1);
        double speed = speeds(stage-1);
        double yaw = utils.get_yaw();
        double yaw_error = yaw - target_yaws(stage-1);
        double yaw_error_sign = yaw_error > 0 ? 1 : -1;
        while(1) {
            yaw = utils.get_yaw();
            while (yaw < -M_PI) yaw += 2*M_PI;
            while (yaw > M_PI) yaw -= 2*M_PI;
            yaw_error = yaw - target_yaws(stage-1);
            while(std::abs(yaw_error) > M_PI * 1.2) {
                if(yaw_error > M_PI * 1.2) {
                    yaw_error -= 2*M_PI;
                } else {
                    yaw_error += 2*M_PI;
                }
            }
            // ROS_INFO("yaw: %.3f, targ: %.3f, yaw_err: %.3f, steer: %.3f, speed: %.3f", yaw, target_yaws(stage-1), yaw_error, steering_angle, speed);
            bool exit_cond;
            if (std::abs(steering_angle) < 0.1) {
                double x, y, yaw;
                utils.get_states(x, y, yaw);
                double dist_sq = std::pow(x - x0, 2) + std::pow(y - y0, 2);
                exit_cond = std::abs(dist_sq - targets(stage-1) * targets(stage-1)) < thresholds(stage-1) * thresholds(stage-1);
                orientation_follow(yaw0, speed);
            } else {
                if(yaw_error_sign < 0) {
                    exit_cond = yaw_error > -thresholds(stage-1);
                } else {
                    exit_cond = yaw_error < thresholds(stage-1);
                }
                // exit_cond = std::abs(yaw_error) < thresholds(stage-1);
                utils.publish_cmd_vel(steering_angle, speed);
            }
            if (exit_cond) {
                ROS_INFO("stage %d completed. yaw error: %.3f", stage, yaw_error);
                stage++;
                if (stage > num_stages) {
                    ROS_INFO("maneuver completed.");
                    break;
                }
                steering_angle = steerings(stage-1);
                speed = speeds(stage-1);
                ROS_INFO("new stage: %d, steer: %.3f, speed: %.3f", stage, steering_angle, speed);
                yaw_error = yaw - target_yaws(stage-1);
                yaw_error_sign = yaw_error > 0 ? 1 : -1;
                continue;
            }
            temp_rate.sleep();
        }
        return 0;
    }

    int move_to(Eigen::VectorXd xs, double thresh=0.1) {
        ros::Rate temp_rate(1/T_park);
        std::cout << "park rate: " << 1/T_park << std::endl;
        mpc.set_up_park(xs);
        int status, i = 0;
        while(1) {
            update_mpc_state();
            status = mpc.update_and_solve_park(xs, thresh*thresh);
            publish_commands();
            if (status == 2 || i>200) {
                break;
            }
            i++;
            temp_rate.sleep();
        }
        return status;
    }
    bool intersection_reached(bool lane = false) {
        static Eigen::Vector2d last_intersection_point = {0, 0};
        static const double INTERSECTION_DISTANCE_THRESHOLD = 0.753; // minimum distance between two intersections
        if(lane) {
            if (utils.stopline)
            {
                mpc.update_current_states(running_x, running_y, running_yaw);
                int closest_idx = mpc.find_closest_waypoint(0, mpc.state_refs.rows()-1);
                int num_index = static_cast<int>(0.15 * mpc.density);
                for (int i = closest_idx; i < closest_idx + num_index; i++) {
                    if (mpc.attribute_cmp(i, mpc.ATTRIBUTE::CROSSWALK) || mpc.attribute_cmp(i, mpc.ATTRIBUTE::DOTTED_CROSSWALK)) {
                        ROS_INFO("detected crosswalk lines, ignoring...");
                        return false;
                    }
                }
                if(check_crosswalk() > 0) {
                    ROS_INFO("crosswalk detected, ignoring...");
                    return false;
                }
                // double x, y, yaw;
                // utils.get_states(x, y, yaw);
                // double dist_sq = std::pow(x - last_intersection_point(0), 2) + std::pow(y - last_intersection_point(1), 2);
                // if (dist_sq < INTERSECTION_DISTANCE_THRESHOLD * INTERSECTION_DISTANCE_THRESHOLD) {
                //     // ROS_INFO("intersection detected, but too close to previous intersection: %.3f, ignoring...", std::sqrt(dist_sq));
                //     return false;
                // }
                // last_intersection_point = {x, y};
                if (!hasGps) {
                    ROS_INFO("intersection detected, relocalizing...");
                    intersection_based_relocalization();
                }
                return true;
            }
        }
        double lookahead_dist = 0.15;
        int num_index = static_cast<int>(lookahead_dist * mpc.density);
        utils.get_states(running_x, running_y, running_yaw);
        mpc.update_current_states(running_x, running_y, running_yaw);
        int target_index = mpc.find_closest_waypoint(0, mpc.state_refs.rows()-1);
        for (int i = 0; i < num_index; i++) {
            // auto attribute_i = mpc.state_attributes(target_index+i); // check 3 waypoints ahead
            // if(attribute_i == mpc.ATTRIBUTE::STOPLINE) {
            if(mpc.attribute_cmp(target_index+i, mpc.ATTRIBUTE::STOPLINE)) {
                // std::cout << "stopline detected at (" << mpc.state_refs(mpc.target_waypoint_index, 0) << ", " << mpc.state_refs(mpc.target_waypoint_index, 1) << ")" << std::endl;
                double x, y, yaw;
                utils.get_states(x, y, yaw);
                double dist_sq = std::pow(x - last_intersection_point(0), 2) + std::pow(y - last_intersection_point(1), 2);
                if (dist_sq < INTERSECTION_DISTANCE_THRESHOLD * INTERSECTION_DISTANCE_THRESHOLD) {
                    // ROS_INFO("intersection detected, but too close to previous intersection: %.3f, ignoring...", std::sqrt(dist_sq));
                    return false;
                }
                last_intersection_point = {x, y};
                return true;
            }
        }
        return false;
    }
    void check_stop_sign() {
        if (!stopsign_flag) {
            int sign_index = utils.object_index(OBJECT::STOPSIGN);
            if(sign_index >= 0) {
                double dist = utils.object_distance(sign_index);
                if (dist < MAX_SIGN_DIST && dist > 0) {
                    std::cout << "stop sign detected at a distance of: " << dist << std::endl;
                    detected_dist = dist;
                    // if(lane) utils.reset_odom();
                    stopsign_flag = STOPSIGN_FLAGS::STOP;
                }
            }
            if (stopsign_flag == STOPSIGN_FLAGS::NONE) {
                sign_index = utils.object_index(OBJECT::LIGHTS);
                if(sign_index >= 0) {
                    double dist = utils.object_distance(sign_index);
                    if (dist < MAX_SIGN_DIST && dist > 0) {
                        std::cout << "traffic light detected at a distance of: " << dist << std::endl;
                        detected_dist = dist;
                        // if(lane) utils.reset_odom();
                        stopsign_flag = STOPSIGN_FLAGS::LIGHT;
                    }
                }
            }
            if (stopsign_flag == STOPSIGN_FLAGS::NONE) {
                sign_index = utils.object_index(OBJECT::PRIORITY);
                if(sign_index >= 0) {
                    double dist = utils.object_distance(sign_index);
                    if (dist < MAX_SIGN_DIST && dist > 0) {
                        std::cout << "prio detected at a distance of: " << dist << std::endl;
                        detected_dist = dist;
                        stopsign_flag = STOPSIGN_FLAGS::PRIO;
                    }
                }
            }
            if (stopsign_flag == STOPSIGN_FLAGS::NONE) {
                sign_index = utils.object_index(OBJECT::ROUNDABOUT);
                if(sign_index >= 0) {
                    double dist = utils.object_distance(sign_index);
                    if (dist < MAX_SIGN_DIST && dist > 0) {
                        std::cout << "roundabout detected at a distance of: " << dist << std::endl;
                        detected_dist = dist;
                        stopsign_flag = STOPSIGN_FLAGS::RDB;
                    }
                }
            }
            if (!hasGps && stopsign_flag != STOPSIGN_FLAGS::NONE) {
                auto sign_pose = utils.estimate_object_pose2d(running_x, running_y, running_yaw, utils.object_box(sign_index), detected_dist, CAMERA_PARAMS);
                if (stopsign_flag == STOPSIGN_FLAGS::RDB) {
                    sign_based_relocalization(sign_pose, ROUNDABOUT_POSES);
                } else {
                    sign_based_relocalization(sign_pose, INTERSECTION_SIGN_POSES);
                }
            }
        }
        // int stopsign_index = utils.object_index(OBJECT::STOPSIGN);
        // if(stopsign_index >= 0) {
        //     dist = utils.object_distance(stopsign_index);
        //     if (ros::Time::now() > cooldown_timer && dist > 0) {
        //         std::cout << "stop sign detected at a distance of: " << dist << std::endl;
        //         // auto box = utils.object_box(stopsign_index);
        //         // auto world_pose = utils.estimate_object_pose2d(utils.get_real_states(), box, dist, CAMERA_PARAMS);
        //         // std::cout << "world pose: " << world_pose << std::endl;
        //         // utils.add_object("stopsign", world_pose);
        //         detected_dist = dist;
        //         change_state(STATE::APPROACHING_INTERSECTION);
        //         continue;
        //     }
        // }
        // int light_index = utils.object_index(OBJECT::LIGHTS);
        // if(light_index >= 0) {
        //     dist = utils.object_distance(light_index);
        //     if (ros::Time::now() > cooldown_timer && dist > 0) {
        //         std::cout << "traffic light detected at a distance of: " << dist << std::endl;
        //         // auto box = utils.object_box(stopsign_index);
        //         // auto world_pose = utils.estimate_object_pose2d(utils.get_real_states(), box, dist, CAMERA_PARAMS);
        //         // std::cout << "world pose: " << world_pose << std::endl;
        //         // utils.add_object("light", world_pose);
        //         detected_dist = dist;
        //         change_state(STATE::APPROACHING_INTERSECTION);
        //         continue;
        //     }
        // }
    }
    int park_sign_detected() {
        int park_index = utils.object_index(OBJECT::PARK);
        if(park_index >= 0) {
            double dist = utils.object_distance(park_index);
            if (dist < MAX_PARK_DIST && dist > 0) {
                detected_dist = dist;
                return park_index;
            }
        }
        return -1;
    }
    double check_crosswalk() {
        static ros::Time crosswalk_cooldown_timer = ros::Time::now();
        static double detected_dist = -1;
        if(crosswalk_cooldown_timer > ros::Time::now()) {
            std::cout << "crosswalk detected previously, cd expire in " << (crosswalk_cooldown_timer - ros::Time::now()).toSec() << "s" << std::endl;
            return 100;
        }
        int crosswalk_index = utils.object_index(OBJECT::CROSSWALK);
        if(crosswalk_index >= 0) {
            detected_dist = utils.object_distance(crosswalk_index);
            if (detected_dist < MAX_CROSSWALK_DIST && detected_dist > 0) {
                double cd = (detected_dist + CROSSWALK_LENGTH) / NORMAL_SPEED * 5/7;
                crosswalk_cooldown_timer = ros::Time::now() + ros::Duration(cd);
                std::cout << "crosswalk detected at a distance of: " << detected_dist << std::endl;
                return detected_dist;
            }
        }
        return -1;
    }
    double check_highway() {
        mpc.update_current_states(running_x, running_y, running_yaw);
        int closest_idx = mpc.find_closest_waypoint(0, mpc.state_refs.rows()-1);
        return mpc.attribute_cmp(closest_idx, mpc.ATTRIBUTE::HIGHWAYLEFT) || mpc.attribute_cmp(closest_idx, mpc.ATTRIBUTE::HIGHWAYRIGHT);
    }
    void pedestrian_detected() {
        int pedestrian_count = 0;
        bool detected;
        if (real) {
            detected = utils.object_index(OBJECT::PEDESTRIAN) >= 0;
        } else {
            detected = utils.object_index(OBJECT::PEDESTRIAN) >= 0 || utils.object_index(OBJECT::HIGHWAYEXIT) >= 0;
        }
        if (detected) {
            while (true) {
                if (real) {
                    detected = utils.object_index(OBJECT::PEDESTRIAN) >= 0;
                } else {
                    detected = utils.object_index(OBJECT::PEDESTRIAN) >= 0 || utils.object_index(OBJECT::HIGHWAYEXIT) >= 0;
                }
                if (detected) {
                    double dist;
                    if (real) dist = utils.object_distance(utils.object_index(OBJECT::HIGHWAYEXIT));
                    else dist = utils.object_distance(utils.object_index(OBJECT::PEDESTRIAN));
                    ROS_INFO("girl detected at a distance of: %.3f", dist);
                    stop_for(STOP_DURATION);
                } else {
                    pedestrian_count ++;
                    ROS_INFO("pedestrian count: %d", pedestrian_count);
                    stop_for(STOP_DURATION/15);
                }
                if (pedestrian_count > 10) break;
            }
            rate->sleep();
        }
    }
    void exit_detected() {
        if (utils.object_index(OBJECT::NOENTRY) >= 0) {
            ROS_INFO("no entry detected, signaling end of mission");
            change_state(STATE::DONE);
        }
    }
    template <std::size_t N>
    int sign_based_relocalization(const Eigen::Vector2d estimated_sign_pose, const std::array<std::array<double, 2>, N> &EMPIRICAL_POSES) {
        int min_index = 0;
        double min_error_sq = 1000;
        for (std::size_t i = 0; i < N; ++i) {
            double error_sq = std::pow(estimated_sign_pose[0] - EMPIRICAL_POSES[i][0], 2) + std::pow(estimated_sign_pose[1] - EMPIRICAL_POSES[i][1], 2);
            if (error_sq < min_error_sq) {
                min_error_sq = error_sq;
                min_index = static_cast<int>(i);
            }
        }
        ROS_INFO("estimated sign pose: (%.3f, %.3f), actual: (%.3f, %.3f), error: (%.3f, %.3f)", estimated_sign_pose[0], estimated_sign_pose[1], EMPIRICAL_POSES[min_index][0], EMPIRICAL_POSES[min_index][1], EMPIRICAL_POSES[min_index][0] - estimated_sign_pose[0], EMPIRICAL_POSES[min_index][1] - estimated_sign_pose[1]);
        // exit(0);
        if (min_error_sq > sign_localization_threshold * sign_localization_threshold) {
            ROS_INFO("error too large: %.3f, relocalization failed...", std::sqrt(min_error_sq));
            return 0;
        } else {
            utils.recalibrate_states(EMPIRICAL_POSES[min_index][0] - estimated_sign_pose[0], EMPIRICAL_POSES[min_index][1] - estimated_sign_pose[1]);
        }
        return 1;
    }
    int intersection_based_relocalization() {
        // stop_for(0.5);
        // check orientation
        double yaw = utils.get_yaw();
        double nearest_direction = mpc.NearestDirection(yaw);
        double yaw_error = nearest_direction - yaw;
        if(yaw_error > M_PI * 1.5) yaw_error -= 2 * M_PI;
        else if(yaw_error < -M_PI * 1.5) yaw_error += 2 * M_PI;
        if(yaw_error > 0.4) {
            ROS_WARN("yaw error too large: %.3f, intersection based relocalization failed...", yaw_error);
            return 0;
        }

        int nearestDirectionIndex = mpc.NearestDirectionIndex(yaw);
        const auto& direction_intersections = (nearestDirectionIndex == 0) ? EAST_FACING_INTERSECTIONS :
                                          (nearestDirectionIndex == 1) ? WEST_FACING_INTERSECTIONS :
                                          (nearestDirectionIndex == 2) ? NORTH_FACING_INTERSECTIONS :
                                                                        SOUTH_FACING_INTERSECTIONS;
        
        static Eigen::Vector2d estimated_position(0, 0);
        utils.get_states(estimated_position(0), estimated_position(1), yaw);
        estimated_position[0] += constant_distance_to_intersection_at_detection * cos(yaw);
        estimated_position[1] += constant_distance_to_intersection_at_detection * sin(yaw);

        double min_error_sq = std::numeric_limits<double>::max();
        int min_index = 0;
        for (size_t i = 0; i < direction_intersections.size(); ++i) {
            double error_sq = std::pow(estimated_position[0] - direction_intersections[i][0], 2) + std::pow(estimated_position[1] - direction_intersections[i][1], 2);
            if (error_sq < min_error_sq) {
                min_error_sq = error_sq;
                min_index = static_cast<int>(i);
            }
        }

        ROS_INFO("intersection based relocalization(): estimated intersection position: (%.3f, %.3f), actual: (%.3f, %.3f), error: (%.3f, %.3f)", estimated_position[0], estimated_position[1], direction_intersections[min_index][0], direction_intersections[min_index][1], direction_intersections[min_index][0] - estimated_position[0], direction_intersections[min_index][1] - estimated_position[1]);
        // exit(0);
        if (min_error_sq < intersection_localization_threshold * intersection_localization_threshold) {
            utils.recalibrate_states(direction_intersections[min_index][0] - estimated_position[0], direction_intersections[min_index][1] - estimated_position[1]);
            return 1; // Successful relocalization
        } else {
            return 0; // Failed to relocalize
        }
    }

    int lane_based_relocalization() {
        double center = utils.center + pixel_center_offset;
        if (center >= 240 && center <= 400) {
            double yaw = utils.get_yaw();
            double nearest_direction = mpc.NearestDirection(yaw);
            double yaw_error = nearest_direction - yaw;
            if(yaw_error > M_PI * 1.5) yaw_error -= 2 * M_PI;
            else if(yaw_error < -M_PI * 1.5) yaw_error += 2 * M_PI;
            if (std::abs(yaw_error) > lane_localization_orientation_threshold * M_PI / 180) {
                // ROS_WARN("lane_based_relocalization(): yaw error too large: %.3f, lane based relocalization failed...", yaw_error);
                return 0;
            }
            double offset = (IMAGE_WIDTH/2 - center) / 80 * LANE_CENTER_TO_EDGE;
            int nearestDirectionIndex = mpc.NearestDirectionIndex(yaw);
            const auto& LANE_CENTERS = (nearestDirectionIndex == 0) ? EAST_FACING_LANE_CENTERS :
                                        (nearestDirectionIndex == 1) ? WEST_FACING_LANE_CENTERS :
                                        (nearestDirectionIndex == 2) ? NORTH_FACING_LANE_CENTERS :
                                                                    SOUTH_FACING_LANE_CENTERS;
            if (nearestDirectionIndex == 0 || nearestDirectionIndex == 2) { // East, 0 || West, 2
                if (nearestDirectionIndex == 2) offset *= -1;
                int min_index = 0;
                double min_error = 1000;
                for (size_t i = 0; i < LANE_CENTERS.size(); i++) {
                    double error = std::abs((LANE_CENTERS[i] - offset) - running_y);
                    if (error < min_error) {
                        min_error = error;
                        min_index = i;
                    }
                }
                if (min_error < LANE_OFFSET) {
                    utils.recalibrate_states(0, min_error);
                    return 1;
                } else {
                    ROS_WARN("lane_based_relocalization(): offset too large: %.3f, lane based relocalization failed...", min_error);
                    return 0;
                }
            } else if (nearestDirectionIndex == 1 || nearestDirectionIndex == 3) { // North, 1 || South, 3
                if (nearestDirectionIndex == 3) offset *= -1;
                int min_index = 0;
                double min_error = 1000;
                for (size_t i = 0; i < LANE_CENTERS.size(); i++) {
                    double error = (LANE_CENTERS[i] + offset) - running_x;
                    if (std::abs(error) < min_error) {
                        min_error = error;
                        min_index = i;
                    }
                }
                // ROS_INFO("center: %.3f, offset: %.3f, running_x: %.3f, min_error: %.3f, closest lane center: %.3f", center, offset, running_x, min_error, Y_ALIGNED_LANE_CENTERS[min_index]);
                if (std::abs(min_error) < LANE_OFFSET) {
                    utils.recalibrate_states(min_error, 0);
                    return 1;
                } else {
                    ROS_WARN("lane_based_relocalization(): offset too large: %.3f, lane based relocalization failed...", min_error);
                    return 0;
                }
            }
        }
        return 0;
    }
    void publish_waypoints() {
        static Eigen::MatrixXd waypoints = Eigen::MatrixXd::Zero(mpc.N, 3);
        mpc.get_current_waypoints(waypoints);
        static std_msgs::Float32MultiArray msg;
        msg.data.clear();
        for (int i = 0; i < waypoints.rows(); ++i) {
            msg.data.push_back(waypoints(i, 0)); // x
            msg.data.push_back(waypoints(i, 1)); // y
        }
        utils.waypoints_pub.publish(msg);
    }
    void lane_follow(double speed = NORMAL_SPEED) {
        if(pubWaypoints) {
            publish_waypoints();
        }
        double steer = utils.get_steering_angle();
        //std::cout << "lanefollowing " << steer<<" " << speed << std::endl;
        if (check_crosswalk() > 0) {
            speed *= cw_speed_ratio;
        }
        if (check_highway() > 0) {
            speed *= hw_speed_ratio;
        }
        utils.publish_cmd_vel(steer, speed);
    }
    void orientation_follow(double orientation, double speed = NORMAL_SPEED) {
        if(pubWaypoints) {
            publish_waypoints();
        }
        double yaw_error = orientation - utils.get_yaw();
        if(yaw_error > M_PI * 1.5) yaw_error -= 2 * M_PI;
        else if(yaw_error < -M_PI * 1.5) yaw_error += 2 * M_PI;
        double steer = - yaw_error * 180 / M_PI * 1;
        if (check_crosswalk() > 0) {
            speed *= cw_speed_ratio;
        }
        if (check_highway() > 0) {
            speed *= hw_speed_ratio;
        }
        utils.publish_cmd_vel(steer, speed);
    }
    void check_car() {
        double dist;
        std::list<int> cars = utils.recent_car_indices;
        // std::cout << "number of cars detected: " << cars.size() << std::endl;
        int car_index = utils.object_index(OBJECT::CAR);
        if(car_index >= 0) { // if car detected
        // for (int car_index: cars) {
            mpc.update_current_states(running_x, running_y, running_yaw);
            int closest_idx = mpc.find_closest_waypoint();
            dist = utils.object_distance(car_index); // compute distance to back of car
            if (dist < MAX_CAR_DIST && dist > 0 && closest_idx >= end_index * 1.2) {
                utils.object_box(car_index, bbox);
                double x, y, yaw;
                utils.get_states(x, y, yaw);
                auto car_pose = utils.estimate_object_pose2d(x, y, yaw, bbox, dist, CAMERA_PARAMS);
                // auto car_pose = utils.detected_cars[car_index];
                printf("estimated car pose: %.3f, %.3f\n", car_pose[0], car_pose[1]);
                // compute distance from detected car to closest waypoint in front of car to assess whether car is in same lane
                double look_ahead_dist = dist * 1.5;
                int look_ahead_index = look_ahead_dist * mpc.density + closest_idx;
                // compute distance from car_pose to waypoint, find closest waypoint and distance
                double min_dist_sq = 1000.;
                int min_index = 0;
                // std::cout << "closest index: " << closest_idx << ", look ahead index: " << look_ahead_index << std::endl;
                for (int i = closest_idx; i < look_ahead_index; i++) {
                    // double dist_sq = (car_pose.head(2) - mpc.state_refs.row(i).head(2)).squaredNorm();
                    if (i >= mpc.state_refs.rows()) {
                        ROS_WARN("check_car(): i exceeds state_refs size, stopping...");
                        break;
                    }
                    double dist_sq = std::pow(car_pose[0] - mpc.state_refs(i, 0), 2) + std::pow(car_pose[1] - mpc.state_refs(i, 1), 2);
                    if (dist_sq < min_dist_sq) {
                        min_dist_sq = dist_sq;
                        min_index = i;
                    }
                }
                double min_dist = std::sqrt(min_dist_sq);
                bool same_lane = false;
                if (min_dist < LANE_OFFSET * 0.35) {
                    same_lane = true;
                }
                std::cout << "min dist between car and closest waypoint: " << min_dist << ", same lane: " << same_lane << std::endl;
                if (same_lane) {
                    int idx = closest_idx + dist * mpc.density * 0.75; // compute index of midpoint between detected car and ego car
                    // int attribute = mpc.state_attributes(idx);
                    // std::cout << "attribute: " << attribute << std::endl;
                    // if (attribute != mpc.ATTRIBUTE::DOTTED && attribute != mpc.ATTRIBUTE::DOTTED_CROSSWALK && attribute != mpc.ATTRIBUTE::HIGHWAYLEFT && attribute != mpc.ATTRIBUTE::HIGHWAYRIGHT) {
                    if (!mpc.attribute_cmp(idx, mpc.ATTRIBUTE::DOTTED) && !mpc.attribute_cmp(idx, mpc.ATTRIBUTE::DOTTED_CROSSWALK) && !mpc.attribute_cmp(idx, mpc.ATTRIBUTE::HIGHWAYLEFT) && !mpc.attribute_cmp(idx, mpc.ATTRIBUTE::HIGHWAYRIGHT)) {
                        if (dist < MAX_TAILING_DIST) {
                            ROS_INFO("detected car is in oneway or non-dotted region, dist = %.3f, stopping...", dist);
                            stop_for(20*T);
                            return;
                        } else {
                            ROS_INFO("car on oneway pretty far and within safety margin, keep tailing: %.3f", dist);
                        }
                    } else {
                        double start_dist = std::max(dist - CAM_TO_CAR_FRONT, MIN_DIST_TO_CAR) - MIN_DIST_TO_CAR;
                        bool right = false;
                        double density = mpc.density;
                        double lane_offset = LANE_OFFSET * 1.5;
                        int end_index_scaler = 1.2;
                        // if (attribute == mpc.ATTRIBUTE::HIGHWAYRIGHT) { // if on right side of highway, overtake on left
                        if (mpc.attribute_cmp(idx, mpc.ATTRIBUTE::HIGHWAYRIGHT)) { // if on right side of highway, overtake on left
                            density *= 1/1.33;
                            end_index_scaler *= 1.5;
                            ROS_INFO("in highway right, overtake on left");
                        }
                        // else if (attribute == mpc.ATTRIBUTE::HIGHWAYLEFT) { // if on left side of highway, overtake on right
                        else if (mpc.attribute_cmp(idx, mpc.ATTRIBUTE::HIGHWAYLEFT)) { // if on left side of highway, overtake on right
                            right = true; 
                            density *= 1/1.33;
                            end_index_scaler *= 1.5;
                            ROS_INFO("in highway left, overtake on right");
                        }
                        if (!lane) {
                            int start_index = closest_idx + static_cast<int>(start_dist * density);
                            
                            end_index = start_index + static_cast<int>((CAR_LENGTH * 2 + MIN_DIST_TO_CAR * 2) * density * end_index_scaler);
                            if (start_index >= mpc.state_refs.rows() || end_index >= mpc.state_refs.rows()) {
                                ROS_WARN("check_car(): start or end index exceeds state_refs size, stopping...");
                                return;
                            };
                            mpc.change_lane(start_index, end_index, right, lane_offset);
                            ROS_INFO("changing lane to the %s in %.3f meters. start pose: (%.2f,%.2f), end: (%.2f, %.2f), cur: (%.2f, %.2f)", start_dist, right ? "right" : "left", mpc.state_refs(start_index, 0), mpc.state_refs(start_index, 1), mpc.state_refs(end_index, 0), mpc.state_refs(end_index, 1), x, y);
                        } else {
                            ROS_INFO("hardcode lane change. proceeding to lane follow until start distance (%.3f) reached...", start_dist);
                            start_dist = std::max(0.0, start_dist - CAR_LENGTH/2);
                            double x0, y0, yaw0;
                            utils.get_states(x0, y0, yaw0);
                            while (true) {
                                double x, y, yaw;
                                utils.get_states(x, y, yaw);
                                double dist_sq = std::pow(x - x0, 2) + std::pow(y - y0, 2);
                                if (dist_sq > start_dist * start_dist) {
                                    ROS_INFO("start point reached, proceeding to lane change...");
                                    break;
                                }
                                // std::cout << "dist thresh: " << start_dist << ", dist: " << std::sqrt(dist_sq) << std::endl;
                                lane_follow();
                                rate->sleep();
                            }
                            int start_index = closest_idx + static_cast<int>((start_dist + CAR_LENGTH) * density);
                            double end_dist = (CAR_LENGTH * 2 + MIN_DIST_TO_CAR * 2) * 0.7 * end_index_scaler;
                            int end_index = start_index + static_cast<int>((end_dist + CAR_LENGTH) * density);
                            if (start_index >= mpc.state_refs.rows() || end_index >= mpc.state_refs.rows()) {
                                ROS_WARN("check_car(): start or end index exceeds state_refs size, stopping...");
                                return;
                            };
                            // double start_yaw_diff = std::atan2(mpc.state_refs(start_index, 1) - y0, mpc.state_refs(start_index, 0) - x0);
                            // // if (std::abs(start_yaw_diff) > 0.4 * M_PI) start_yaw_diff = 0;
                            
                            // double end_yaw_diff = std::atan2(mpc.state_refs(end_index, 1) - mpc.state_refs(start_index, 1), mpc.state_refs(end_index, 0) - mpc.state_refs(start_index, 0));
                            // // if (std::abs(end_yaw_diff) > 0.4 * M_PI) end_yaw_diff = 0;

                            double start_yaw = mpc.state_refs(start_index, 2);
                            double end_yaw = mpc.state_refs(end_index, 2);
                            ROS_INFO("start yaw: %.3f, end yaw: %.3f", start_yaw, end_yaw);
                            double start_yaw_diff = start_yaw - yaw0;
                            double end_yaw_diff = end_yaw - start_yaw;
                            while (start_yaw_diff > M_PI) start_yaw_diff -= 2*M_PI;
                            while (start_yaw_diff < -M_PI) start_yaw_diff += 2*M_PI;
                            while (end_yaw_diff > M_PI) end_yaw_diff -= 2*M_PI;
                            while (end_yaw_diff < -M_PI) end_yaw_diff += 2*M_PI;

                            ROS_INFO("start point: <%.3f, %.3f>, start index: %d, end point: <%.3f, %.3f>, end index: %d", mpc.state_refs(start_index, 0), mpc.state_refs(start_index, 1), start_index, mpc.state_refs(end_index, 0), mpc.state_refs(end_index, 1), end_index);
                            ROS_INFO("yaw0: %.3f, start_yaw_diff: %.3f, end_yaw_diff: %.3f", yaw0, start_yaw_diff, end_yaw_diff);

                            utils.get_states(x0, y0, yaw0);
                            change_lane_hardcode(right, start_yaw_diff);
                            ROS_INFO("lane change completed, proceeding to lane follow until end distance (%.3f) reached...", end_dist);
                            while (true) {
                                double x, y, yaw;
                                utils.get_states(x, y, yaw);
                                double dist_sq = std::pow(x - x0, 2) + std::pow(y - y0, 2);
                                if (dist_sq > end_dist * end_dist) {
                                    ROS_INFO("end point reached, changing back to original lane...");
                                    break;
                                }
                                // std::cout << "dist thresh: " << end_dist << ", dist: " << std::sqrt(dist_sq) << std::endl;
                                lane_follow();
                                rate->sleep();
                            }
                            change_lane_hardcode(!right, end_yaw_diff);
                            ROS_INFO("maneuver completed, proceeding to lane follow...");
                        }
                    }
                } else {
                    // ROS_INFO("detected car is not in the same lane, ignoring...");
                }
            }
        }
    }
};

void StateMachine::update_mpc_state() {
    double x, y, yaw;
    utils.get_states(x, y, yaw);
    while(!mpc.update_current_states(x, y, yaw)) {
        ROS_WARN("update_current_states failed, stopping briefly...");
        stop_for(STOP_DURATION / 3);
        utils.get_states(x, y, yaw);
    }
    if(debug) {
        ;
    }
}
void StateMachine::solve(bool safety_check) {
    // auto start = std::chrono::high_resolution_clock::now();

    int status = mpc.update_and_solve(mpc.x_current, safety_check);
    publish_commands();
    if (debug) {
        ;
    }
    // auto stop = std::chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
    // ROS_INFO("Solve time: %f ms", duration.count()/1000.0);
}
void StateMachine::publish_commands() {
    /*
    Publishes the commands of the MPC controller to the car
    */

    if(pubWaypoints) {
        publish_waypoints();
    }
    
    double steer = -mpc.u_current[1] * 180 / M_PI; // convert to degrees
    double speed = mpc.u_current[0];
    // ROS_INFO("publish_commands: steer: %.3f, speed: %.3f", steer, speed);
    // steer = 23;
    // speed = 0.573;
    utils.publish_cmd_vel(steer, speed);
}
void StateMachine::change_state(STATE new_state) {
    std::cout << "Changing from " << state_names[state] << " to " << state_names[new_state] << std::endl;
    state = new_state;
}

void StateMachine::run() {
    static ros::Time overtake_cd = ros::Time::now();
    static bool wrong_lane = false;
    ROS_INFO("start running");
    double running_x, running_y, running_yaw;
    while (ros::ok()) {
        if (sign) {
            pedestrian_detected();
            exit_detected();
        }
        utils.get_states(running_x, running_y, running_yaw);
        // if (mpc.target_waypoint_index >= mpc.num_waypoints -1) change_state(STATE::DONE);
        if (state == STATE::MOVING) {
            if(intersection_reached()) {
                if(sign) {
                    if (stopsign_flag == STOPSIGN_FLAGS::STOP || stopsign_flag == STOPSIGN_FLAGS::LIGHT) {
                        stopsign_flag = STOPSIGN_FLAGS::NONE;
                        change_state(STATE::WAITING_FOR_STOPSIGN);
                        continue;
                    } else if (stopsign_flag == STOPSIGN_FLAGS::PRIO) {
                        stopsign_flag = STOPSIGN_FLAGS::NONE;
                        ROS_INFO("priority sign detected, keep moving...");
                    }
                } else {
                    // change_state(STATE::WAITING_FOR_STOPSIGN);
                    // continue;
                }
            }
            if (sign) {
                check_stop_sign();
                int park_index = park_sign_detected();
                if(park_index>=0 && park_count < 1) {
                    ROS_INFO("parking sign detected, proceeding to parking...");
                    change_state(STATE::PARKING);
                    park_count++;
                    continue;
                }
                check_car();    
            }
            update_mpc_state();
            double error_sq = (mpc.x_current.head(2) - destination).squaredNorm();
            if (error_sq < TOLERANCE_SQUARED && mpc.target_waypoint_index >= mpc.state_refs.rows() * 0.9) {
                change_state(STATE::DONE);
            }
            solve();
            rate->sleep();
            continue;
        } else if (state == STATE::APPROACHING_INTERSECTION) {
            double offset = std::max(0.0, detected_dist - MIN_SIGN_DIST) * 0.7; //???
            double x0, y0, yaw0;
            utils.get_states(x0, y0, yaw0);
            // utils.reset_odom();
            double orientation = mpc.NearestDirection(utils.get_yaw());
            ROS_INFO("sign detected at a distance of: %.3f, offset: %.3f, resetting odom...", detected_dist, offset);
            while(1) {
                double x, y, yaw;
                utils.get_states(x, y, yaw);
                double norm_sq = std::pow(x - x0, 2) + std::pow(y - y0, 2);
                if (norm_sq >= offset * offset)
                {
                    ROS_INFO("intersection reached, stopping...");
                    utils.publish_cmd_vel(0.0, 0.0);
                    break;
                }
                if (lane) {
                    if (norm_sq > offset * offset * 0.2) {
                        // double yaw_error = orientation - utils.get_yaw();
                        // if(yaw_error > M_PI * 1.5) yaw_error -= 2 * M_PI;
                        // else if(yaw_error < -M_PI * 1.5) yaw_error += 2 * M_PI;
                        // double steer = - yaw_error * 180 / M_PI * 1;
                        // // std::cout << "yaw_error: " << yaw_error << ", steer: " << steer << std::endl;
                        // utils.publish_cmd_vel(steer, 0.175);
                        orientation_follow(orientation);
                    } else {
                        lane_follow();
                    }
                } else {
                    mpc.update_current_states(x, y, yaw);
                    solve();
                }
                rate->sleep();
            }
            if (stopsign_flag == STOPSIGN_FLAGS::STOP || stopsign_flag == STOPSIGN_FLAGS::LIGHT) {
                change_state(STATE::WAITING_FOR_STOPSIGN);
            } else {
                change_state(STATE::INTERSECTION_MANEUVERING);
            }
            stopsign_flag = 0;
            continue;
        } else if (state == STATE::LANE_FOLLOWING) {
            static bool use_stopline = true;
            if (use_stopline) {
                if(intersection_reached(true)) {
                    if(stopsign_flag == STOPSIGN_FLAGS::STOP || stopsign_flag == STOPSIGN_FLAGS::LIGHT) {
                        stopsign_flag = STOPSIGN_FLAGS::NONE;
                        // change_state(STATE::WAITING_FOR_STOPSIGN);
                        ROS_INFO("stop sign detected, stopping for %.3f seconds...", STOP_DURATION);
                        stop_for(STOP_DURATION);
                        change_state(STATE::INTERSECTION_MANEUVERING);
                        continue;
                    } else if(stopsign_flag == STOPSIGN_FLAGS::PRIO) {
                        ROS_INFO("priority detected. transitioning to intersection maneuvering");
                        stopsign_flag = STOPSIGN_FLAGS::NONE;
                        change_state(STATE::INTERSECTION_MANEUVERING);
                        continue;
                    } else if(stopsign_flag == STOPSIGN_FLAGS::RDB) {
                        ROS_INFO("roundabout detected. using mpc...");
                        stopsign_flag = STOPSIGN_FLAGS::NONE;
                        mpc.update_current_states(running_x, running_y, running_yaw, false);
                        solve(false);
                        rate->sleep();
                        continue;
                    } else {
                        ROS_WARN("intersection reached but no sign detected, using mpc...");
                        stopsign_flag = STOPSIGN_FLAGS::NONE;
                        mpc.update_current_states(running_x, running_y, running_yaw, false);
                        solve(false);
                        rate->sleep();
                        continue;
                    }
                }
            } else {
                if (stopsign_flag) {
                    ROS_INFO("stop sign detected, but stopline detection disabled, using distance to gauge intersection");
                    change_state(STATE::APPROACHING_INTERSECTION);
                    continue;
                }
            }
            if (sign) {
                check_stop_sign();
                int park_index = park_sign_detected();
                if(park_index>=0 && park_count < 1) {
                    change_state(STATE::PARKING);
                    park_count++;
                    continue;
                }
                check_car();
            }
            mpc.update_current_states(running_x, running_y, running_yaw, false);
            int closest_idx = mpc.find_closest_waypoint();
            if (!hasGps) {
                static int lane_relocalization_semaphore = 0;
                lane_relocalization_semaphore++;
                if (lane_relocalization_semaphore >= 5) {
                    if (lane_based_relocalization()) ROS_INFO("lane relocalization successful");
                    lane_relocalization_semaphore = 0;
                }
            }
            if(mpc.is_not_detectable(closest_idx)) {
                std::cout << "non-detectable area, using mpc" << std::endl;
                solve(false);
            } else {
                mpc.target_waypoint_index = closest_idx + 1;
                // std::cout << "detectable area, using lane follow" << std::endl;
                if(check_crosswalk() > 0) {
                    orientation_follow(mpc.NearestDirection(utils.get_yaw()));
                } else {
                    lane_follow();
                }
            }
            rate->sleep();
            continue;
        } else if (state == STATE::WAITING_FOR_STOPSIGN) {
            stop_for(STOP_DURATION);
            if (lane) {
                change_state(STATE::INTERSECTION_MANEUVERING);
            } else {
                change_state(STATE::MOVING);
            }
        } else if (state == STATE::WAITING_FOR_LIGHT) {
            stop_for(STOP_DURATION);
            if(lane) change_state(STATE::LANE_FOLLOWING);
            else change_state(STATE::MOVING);
        } else if (state == STATE::PARKING) {
            stop_for(STOP_DURATION/2);
            double offset_thresh = 0.1;
            double base_offset = detected_dist + PARKING_SPOT_LENGTH * 1.5 + offset_thresh;
            double offset = base_offset;
            ROS_INFO("park sign detected at a distance of: %.3f, parking offset is: %.3f", detected_dist, offset);
            // base_offset = 0;
            right_park = true;
            bool hard_code = true;
            int target_spot = 0;
            if (!lane) {
                double orientation = mpc.NearestDirection(utils.get_yaw());
                ROS_INFO("orientation: %.3f", orientation);
                double x0, y0, yaw0;
                utils.get_states(x0, y0, yaw0);
                int park_index = utils.object_index(OBJECT::PARK);
                if(park_index >= 0) {
                    double dist = utils.object_distance(park_index);
                } else {
                    ROS_WARN("parking sign invalid... returning to STATE::MOVING");
                    change_state(STATE::MOVING);
                }
                if (!hasGps) {
                    auto park_sign_pose = utils.estimate_object_pose2d(x0, y0, yaw0, utils.object_box(park_index), detected_dist, CAMERA_PARAMS);
                    int success = sign_based_relocalization(park_sign_pose, PARKING_SIGN_POSES);
                }
                std::cout << "target spot: " << target_spot << std::endl;
                while(1) {
                    pedestrian_detected();
                    exit_detected();
                    // check utils.recent_car_indices
                    // std::cout << "recent car indices size: " << utils.recent_car_indices.size() << std::endl;
                    std::list<int> cars = utils.recent_car_indices;
                    // std::cout << "number of cars detected: " << cars.size() << std::endl;
                    // iterate through all cars and check if any are in the parking spot
                    bool changed = false;
                    bool car_in_spot = false;
                    while(1) {
                        for (int i : cars) {
                        // for (int i = 0; i < cars.size(); i++) {
                            // auto box = utils.object_box(cars[i]);
                            // double dist = utils.object_distance(cars[i]);
                            // double x, y, yaw;
                            // utils.get_states(x, y, yaw);
                            // auto world_pose = utils.estimate_object_pose2d(x, y, yaw, box, dist, CAMERA_PARAMS);
                            Eigen::Vector2d world_pose = utils.detected_cars[i];
                            // std::cout << "world pose: " << world_pose[0] << ", " << world_pose[1] << std::endl;
                            Eigen::Vector2d spot = PARKING_SPOTS[target_spot];
                            // double detected_x = world_pose[0];
                            // double detected_y = world_pose[1];
                            // double error = (detected_x - PARKING_SPOTS[target_spot][0])*(detected_x - PARKING_SPOTS[target_spot][0]) + (detected_y - PARKING_SPOTS[target_spot][1])*(detected_y - PARKING_SPOTS[target_spot][1]);
                            double error_sq = (world_pose - spot).squaredNorm();
                            double error_threshold_sq = 0.04;
                            if (error_sq < error_threshold_sq) {
                                car_in_spot = true;
                                std::cout << "car detected in spot: " << target_spot << std::endl;
                                target_spot++;
                                changed = true;
                                break;
                            }
                            car_in_spot = false;
                        }
                        if (!car_in_spot) break;
                    }
                    if (changed) {
                        // xs[0] = base_offset + target_spot/2 * PARKING_SPOT_LENGTH;
                        offset = base_offset + target_spot / 2 * PARKING_SPOT_LENGTH;
                        right_park = target_spot % 2 == 0;
                        ROS_INFO("car in spot, changing to target spot %d at (%.3f, %.3f), right: %s", target_spot, PARKING_SPOTS[target_spot][0], PARKING_SPOTS[target_spot][1], right_park ? "true" : "false");
                        // mpc.set_up_park(xs);
                    }
                    // update_mpc_state();
                    // status = mpc.update_and_solve_park(xs, offset_thresh*offset_thresh);
                    // publish_commands();
                    // if (status == 2 || i>200) {
                    //     break;
                    // }
                    // i++;
                    // temp_rate.sleep();
                    double x, y, yaw;
                    utils.get_states(x, y, yaw);
                    double norm_sq = std::pow(x - x0, 2) + std::pow(y - y0, 2);
                    if (norm_sq >= offset * offset)
                    {
                        ROS_INFO("x offset reached: (%.2f, %.2f), ready for parking maneuver...", x, y);
                        stop_for(STOP_DURATION/2);
                        break;
                    }
                    orientation_follow(orientation);
                    // if (norm_sq > offset * offset * 0.2) {
                    //     orientation_follow(orientation);
                    // } else {
                    //     update_mpc_state();
                    //     solve();
                    // }
                    rate->sleep();
                }
            } else {
                double orientation = mpc.NearestDirection(utils.get_yaw());
                ROS_INFO("orientation: %.3f", orientation);
                double x0, y0, yaw0;
                utils.get_states(x0, y0, yaw0);
                // utils.reset_odom();
                while(1) {
                    pedestrian_detected();
                    exit_detected();
                    double x, y, yaw;
                    utils.get_states(x, y, yaw);
                    double norm_sq = std::pow(x - x0, 2) + std::pow(y - y0, 2);
                    ROS_INFO("norm: %.3f", std::sqrt(norm_sq));
                    if (norm_sq >= offset * offset)
                    {
                        ROS_INFO("parking spot reached, stopping...");
                        utils.publish_cmd_vel(0.0, 0.0);
                        break;
                    }
                    if (norm_sq > offset * offset * 0.2) {
                        orientation_follow(orientation);
                    } else {
                        lane_follow();
                    }
                    rate->sleep();
                }
            }
            stop_for(STOP_DURATION/2);
            if (hard_code) {
                // right_park = true; //temp
                double orientation = mpc.NearestDirection(utils.get_yaw());
                double x, y, yaw;
                utils.get_states(x, y, yaw);
                double initial_y_error = y - (PARKING_SPOTS[target_spot][1] + PARKING_SPOT_WIDTH * (right_park ? 1 : -1));
                double initial_yaw_error = orientation - yaw;
                // ROS_INFO("initial y error: %.3f, initial yaw error: %.3f", initial_y_error, initial_yaw_error);
                // exit(0);
                parking_maneuver_hardcode(right_park, false, 1/T_park, initial_y_error, initial_yaw_error);
            } else {
                xs << -0.63, -0.35, 0. , 0., 0.;
                ROS_INFO("moving to: %.3f, %.3f, %.3f", xs[0], xs[1], xs[2]);
                move_to(xs);
            }
            double x, y, yaw;
            utils.get_states(x, y, yaw);
            double x_error = x - PARKING_SPOTS[target_spot][0];
            if (std::abs(x_error) > 0.15) {
                double orientation = mpc.NearestDirection(utils.get_yaw());
                ROS_INFO("parked but x offset too large: %.3f, adjusting... orientation: %.3f", x_error, orientation);
                double x0, y0, yaw0;
                utils.get_states(x0, y0, yaw0);
                while(1) {
                    x_error = x - PARKING_SPOTS[target_spot][0];
                    pedestrian_detected();
                    exit_detected();
                    utils.get_states(x, y, yaw);
                    double norm_sq = std::pow(x - x0, 2) + std::pow(y - y0, 2);
                    if (x_error > 0 && x_error < 0.05 || x_error < 0 && x_error > -0.05)
                    {
                        ROS_INFO("parking spot reached, stopping...");
                        utils.publish_cmd_vel(0.0, 0.0);
                        break;
                    }
                    orientation_follow(orientation);
                    rate->sleep();
                }
            }
            change_state(STATE::PARKED);
        } else if (state == STATE::PARKED) {
            stop_for(STOP_DURATION);
            change_state(STATE::EXITING_PARKING);
        } else if (state == STATE::EXITING_PARKING) {
            bool hard_code = true;
            if (hard_code) {
                // right_park = true;
                parking_maneuver_hardcode(right_park, true, 1/T_park);
            } else {
                xs << -0.2, 0., 0. , 0., 0.;
                ROS_INFO("moving back for %.3f", xs[0]);
                move_to(xs, 0.05);
                xs << 0.63, 0.4, 0., 0., 0.;
                move_to(xs, 0.15);
            }
            if (lane) {
                change_state(STATE::LANE_FOLLOWING);
            } else {
                change_state(STATE::MOVING);
            }
        } else if (state == STATE::INTERSECTION_MANEUVERING) {
            double x, y, yaw;
            utils.get_states(x, y , yaw);
            mpc.update_current_states(x, y, yaw, false);
            int closest_index = mpc.find_closest_waypoint();
            int look_ahead_index = closest_index + mpc.density * 1.5;
            // compute the vector from the current position to the closest waypoint and the angle of the vector
            auto start_vector = mpc.state_refs.row(closest_index).head(2) - mpc.state_refs.row(closest_index-8).head(2);
            auto end_vector = mpc.state_refs.row(look_ahead_index).head(2) - mpc.state_refs.row(look_ahead_index+8).head(2);
            double cross_product = start_vector[0] * end_vector[1] - start_vector[1] * end_vector[0];
            double normalized_cross_product = cross_product / (start_vector.norm() * end_vector.norm());
            int direction_estimate = 0; // 0: left, 1: straight, 2: right
            if (normalized_cross_product > 0.75) {
                direction_estimate = MANEUVER_DIRECTION::RIGHT;
            } else if (normalized_cross_product < -0.75) {
                direction_estimate = MANEUVER_DIRECTION::LEFT;
            } else {
                direction_estimate = MANEUVER_DIRECTION::STRAIGHT;
            }
            ROS_INFO("intersection Maneuvering: direction estimate: %s", MANEUVER_DIRECTIONS[direction_estimate].c_str());
            maneuver_direction = maneuver_indices[maneuver_index];
            if (maneuver_direction != direction_estimate) {
                ROS_WARN("maneuver direction mismatch, expected: %s, detected: %s", MANEUVER_DIRECTIONS[maneuver_direction].c_str(), MANEUVER_DIRECTIONS[direction_estimate].c_str());
                maneuver_direction = direction_estimate;
            }
            maneuver_index++;
            bool use_mpc_for_maneuvering = false;
            if (use_mpc_for_maneuvering) {
                mpc.target_waypoint_index = 0;
                Eigen::MatrixXd* state_refs_ptr;
                if (maneuver_direction == 0) {
                    state_refs_ptr = &mpc.left_turn_states;
                } else if (maneuver_direction == 1) {
                    state_refs_ptr = &mpc.straight_states;
                } else if (maneuver_direction == 2) {
                    state_refs_ptr = &mpc.right_turn_states;
                } else {
                    ROS_INFO("invalid maneuver direction: %d", maneuver_direction);
                    change_state(STATE::MOVING);
                    continue;
                }
                double x0, y0, yaw0;
                utils.get_states(x0, y0, yaw0);
                mpc.frame1 << x0, y0, yaw0;
                mpc.frame2 = state_refs_ptr->row(0);
                mpc.frame1[2] = mpc.NearestDirection(mpc.frame2[2]);
                std::cout << "frame1: " << mpc.frame1 << ",\n frame2: " << mpc.frame2 << std::endl;
                int last_history = mpc.last_waypoint_index;
                mpc.last_waypoint_index = 0;
                while(1) {
                    pedestrian_detected();
                    exit_detected();
                    update_mpc_state();
                    mpc.transform_point(mpc.x_current, mpc.x_current_transformed, mpc.frame1, mpc.frame2);
                    double yaw_frame2 = mpc.frame2[2];
                    while (mpc.x_current_transformed[2] - yaw_frame2 > M_PI) {
                        mpc.x_current_transformed[2] -= 2 * M_PI;
                    }
                    while (mpc.x_current_transformed[2] - yaw_frame2 < -M_PI) {
                        mpc.x_current_transformed[2] += 2 * M_PI;
                    }
                    double error_sq = (mpc.x_current_transformed - state_refs_ptr->row(state_refs_ptr->rows()-1).transpose()).squaredNorm();
                    if (error_sq < TOLERANCE_SQUARED) {
                        break;
                    }
                    // ROS_INFO("error_sq: %.3f", error_sq);
                    int status = mpc.update_and_solve(mpc.x_current_transformed, maneuver_direction);
                    publish_commands();
                    rate->sleep();
                }
                mpc.last_waypoint_index = last_history;
            } else {
                double x0, y0, yaw0;
                utils.get_states(x0, y0, yaw0);
                yaw0 = mpc.NearestDirection(utils.get_yaw());
                static const double directions[5] = {0, M_PI / 2, M_PI, 3 * M_PI / 2, 2 * M_PI};
                int direction_index = mpc.NearestDirectionIndex(yaw0);
                int target_index = (direction_index - (maneuver_direction-1)) % 4;
                if(target_index < 0) target_index += 4;
                double target_yaw = directions[target_index];
                while(target_yaw > M_PI) target_yaw -= 2*M_PI;
                while(target_yaw < -M_PI) target_yaw += 2*M_PI;
                // ROS_INFO("target_yaw: %.3f, cur_yaw: %.3f, direction_index: %d, target_index: %d", target_yaw, yaw0, direction_index, target_index);
                static const std::array<Eigen::Matrix2d, 4> rotation_matrices = {
                    Eigen::Matrix2d::Identity(),
                    (Eigen::Matrix2d() << 0, -1, 1, 0).finished(), 
                    (Eigen::Matrix2d() << -1, 0, 0, -1).finished(),
                    (Eigen::Matrix2d() << 0, 1, -1, 0).finished()
                };
                Eigen::Vector2d odomFrame = {0, 0};
                Eigen::Vector2d transformedFrame = {0, 0};
                utils.setIntersectionDecision(maneuver_direction);
                while(true) {
                    pedestrian_detected();
                    exit_detected();
                    double x, y, yaw;
                    utils.get_states(x, y, yaw);
                    if (maneuver_direction == MANEUVER_DIRECTION::STRAIGHT) {
                        double distance_traveled = std::sqrt(std::pow(x - x0, 2) + std::pow(y - y0, 2));
                        // ROS_INFO("straight maneuver, distance traveled: %.3f", distance_traveled);
                        if (distance_traveled > straight_trajectory_threshold) {
                            break;
                        }
                        odomFrame << x - x0, y - y0;
                        transformedFrame = odomFrame.transpose() * rotation_matrices[direction_index];
                        double referenceY = utils.computeTrajectory(transformedFrame[0]);
                        double error = (referenceY - transformedFrame[1]);
                        double steer = -utils.computeTrajectoryPid(error);
                        // ROS_INFO("target_yaw: %2f, cur_yaw: %2f, yaw_err: %.3f, tar y: %.3f, cur y: %.3f, err: %.3f, steer: %.3f", target_yaw, utils.get_yaw(), yaw_error, referenceY, transformedFrame[1], error, steer);
                        mpc.target_waypoint_index++;
                        utils.publish_cmd_vel(steer, 0.25);
                        rate->sleep();
                    } else {
                        double yaw_error = target_yaw - utils.get_yaw();
                        while(yaw_error > M_PI * 1.5) yaw_error -= 2*M_PI;
                        while(yaw_error < -M_PI * 1.5) yaw_error += 2*M_PI;
                        // ROS_INFO("target_yaw: %.3f, cur_yaw: %.3f, yaw_err: %.3f", target_yaw, utils.get_yaw(), yaw_error);
                        if (std::abs(yaw_error) < left_trajectory_threshold) {
                            break;
                        }
                        // odomFrame << x - x0, y - y0;
                        // transformedFrame = odomFrame.transpose() * rotation_matrices[direction_index];
                        // double referenceY = utils.computeTrajectory(transformedFrame[0]);
                        // double error = (referenceY - transformedFrame[1]);
                        // double steer = -utils.computeTrajectoryPid(error);
                        // // ROS_INFO("target_yaw: %2f, cur_yaw: %2f, yaw_err: %.3f, tar y: %.3f, cur y: %.3f, err: %.3f, steer: %.3f", target_yaw, utils.get_yaw(), yaw_error, referenceY, transformedFrame[1], error, steer);
                        // mpc.target_waypoint_index++;
                        // utils.publish_cmd_vel(steer, 0.25);
                        // rate->sleep();
                        ROS_INFO("intersection maneuvering: using mpc.");
                        mpc.update_current_states(x, y, yaw, false);
                        solve(false);
                        rate->sleep();
                    }
                    
                }
            }
            utils.get_states(running_x, running_y, running_yaw);
            mpc.update_current_states(running_x, running_y, running_yaw, false);
            int index = mpc.find_closest_waypoint(mpc.last_waypoint_index, mpc.last_waypoint_index + 3.0 * mpc.density);
            mpc.last_waypoint_index = index;
            mpc.target_waypoint_index = index;
            change_state(STATE::LANE_FOLLOWING);
        } else if (state == STATE::INIT) {
            if (dashboard) {
                utils.publish_cmd_vel(0, 0);
                rate->sleep();
            } else {
                start();
            }
            // change_state(STATE::INTERSECTION_MANEUVERING);
            // change_state(STATE::PARKING);
        } else if (state == STATE::DONE) {
            std::cout << "Done" << std::endl;
            utils.stop_car();
            // if (debug) {
            //     mpc.computeStats(357);
            // }
            break;
        } else if (state == STATE::KEYBOARD_CONTROL) {
            // Constants for steering and speed
            const double STEERING_INCREMENT = 2;  
            const double VELOCITY_INCREMENT = 0.05;   
            const double MAX_VELOCITY = 0.45;     
            const double MIN_VELOCITY = -0.45;    
            const double HARD_MAX_STEERING = 25.0;
            const double STEERING_DECAY = 1.25;
            const double VELOCITY_DECAY = 0; //0.0025;
            double velocity = 0.0;
            double steering_angle = 0.0;

            // Initialize ncurses mode
            initscr();
            cbreak();
            noecho();
            keypad(stdscr, TRUE);  // Enable keyboard mapping
            timeout(100);          // Non-blocking delay to allow continuous updates

            // Information display
            printw("Use 'w' and 's' to increase or decrease speed.\n");
            printw("Use 'a' and 'd' to control steering.\n");
            printw("'q' to quit.\n");

            int ch;
            bool running = true;

            while (running) {
                ch = getch();
                
                switch (ch) {
                    case 'w':
                        velocity += VELOCITY_INCREMENT;
                        if (velocity > MAX_VELOCITY) velocity = MAX_VELOCITY;
                        break;
                    case 's':
                        velocity -= VELOCITY_INCREMENT;
                        if (velocity < MIN_VELOCITY) velocity = MIN_VELOCITY;
                        break;
                    case 'a':
                        steering_angle -= STEERING_INCREMENT;
                        if (steering_angle < -HARD_MAX_STEERING) steering_angle = -HARD_MAX_STEERING;
                        break;
                    case 'd':
                        steering_angle += STEERING_INCREMENT;
                        if (steering_angle > HARD_MAX_STEERING) steering_angle = HARD_MAX_STEERING;
                        break;
                    case 'b':
                        velocity = 0;
                        break;
                    case 'q':
                        running = false;
                        break;
                    default:
                        // Gradually return the steering towards zero if no steering keys are pressed
                        if (steering_angle > 0) {
                            steering_angle -= STEERING_DECAY;
                            if (steering_angle < 0) steering_angle = 0;
                        } else if (steering_angle < 0) {
                            steering_angle += STEERING_DECAY;
                            if (steering_angle > 0) steering_angle = 0;
                        }
                        if (velocity > 0) {
                            velocity -= VELOCITY_DECAY;
                            if (velocity < 0) velocity = 0;
                        } else if (velocity < 0) {
                            velocity += VELOCITY_DECAY;
                            if (velocity > 0) velocity = 0;
                        }
                        break;
                }

                // Clear previous outputs
                clear();
                printw("Velocity: %f\n", velocity);
                printw("Steering angle: %f\n", steering_angle);
                printw("Press 'q' to quit.");
                utils.publish_cmd_vel(steering_angle, velocity);

            }

            // Clean up ncurses
            endwin();
            utils.stop_car();
            change_state(STATE::INIT);
        }
    }
}


StateMachine *globalStateMachinePtr = nullptr;
void signalHandler(int signum) {
    if (globalStateMachinePtr) {
        globalStateMachinePtr->utils.stop_car();
        //globalStateMachinePtr->mpc.computeStats(357);
        //globalStateMachinePtr->utils.print_detected_cars();

        globalStateMachinePtr->call_trigger_service();
    }

    ros::shutdown();
    exit(signum);
}

using json = nlohmann::json;
int main(int argc, char **argv) {
    // std::string dir = Optimizer::getSourceDirectory();
    // dir.replace(dir.rfind("src"), 3, "scripts");
    // std::string file_path = dir + "/" + "config/mpc_config2.json";
    // std::ifstream file(file_path);
    // if(!file.is_open()) {
    //     std::cout << "Failed to open file: " << file_path << std::endl;
    //     exit(1);
    // }
    // json j;
    // file >> j;

    std::cout.precision(3);
    //create anonymous node handle
    ros::init(argc, argv, "mpc_node", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    double T, v_ref, T_park;
    int N;
    bool sign, ekf, lane, real;
    std::string name;
    std::string nodeName = ros::this_node::getName();
    std::cout << "node name: " << nodeName << std::endl;
    bool success = nh.getParam(nodeName + "/lane", lane) && nh.getParam(nodeName+"/ekf", ekf) && nh.getParam(nodeName+"/sign", sign) && nh.getParam("T", T) && nh.getParam("N", N) && nh.getParam("constraints/v_ref", v_ref);
    double x0, y0, yaw0, vref;
    success = success && nh.getParam(nodeName+"/name", name) && nh.getParam(nodeName+"/vref", vref) && nh.getParam(nodeName+"/x0", x0) && nh.getParam(nodeName+"/y0", y0) && nh.getParam(nodeName+"/yaw0", yaw0);
    success = success && nh.getParam("/T_park", T_park);
    success = success && nh.getParam(nodeName+"/real", real);
    if (!success) {
        std::cout << "Failed to get parameters" << std::endl;
        T = 0.100;
        N = 40;
        v_ref = 0.25;
        sign = false;
        ekf = false;
        lane = false;
        exit(1);
    } else {
        std::cout << "Successfully loaded parameters" << std::endl;
    }
    std::cout << "ekf: " << ekf << ", sign: " << sign << ", T: " << T << ", N: " << N << ", vref: " << vref << ", real: " << real << std::endl;
    StateMachine sm(nh, T, N, vref, sign, ekf, lane, T_park, name, x0, y0, yaw0, real);

    globalStateMachinePtr = &sm;
    signal(SIGINT, signalHandler);

    // std::thread t(&StateMachine::run, &sm);
    std::thread t2(&Utility::spin, &sm.utils);
    
    sm.run();

    // t.join();
    t2.join();
    std::cout << "threads joined" << std::endl;
    return 0;
}
