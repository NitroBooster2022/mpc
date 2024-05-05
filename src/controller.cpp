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
    nh(nh_), utils(nh, real, x_init, y_init, yaw_init, sign, ekf, lane, robot_name), mpc(T,N,v_ref), cooldown_timer(ros::Time::now()), xs(5),
    state(STATE::INIT), sign(sign), ekf(ekf), lane(lane), T_park(T_park), T(T), end_index(0), real(real)
    {
        ros::ServiceClient waypoints_client = nh.serviceClient<utils::waypoints>("/waypoint_path");
        utils::waypoints srv;
        std::string pathName;
        if(!nh.getParam("/pathName", pathName)) {
            ROS_ERROR("Failed to get param 'pathName'");
            pathName = "path1";
        }
        nh.param("/dashboard", dashboard, true);
        nh.param("/kb", keyboardControl, false);
        nh.param("/gps", hasGps, false);
        if (keyboardControl) {
            std::cout << "keyboard control enabled" << std::endl;
            change_state(STATE::KEYBOARD_CONTROL);
        }
        srv.request.pathName = pathName;
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
        x0 = {0, 0, 0};
        destination = mpc.state_refs.row(mpc.state_refs.rows()-1).head(2);
        ROS_INFO("destination: %.3f, %.3f", destination(0), destination(1));
        start_trigger = nh.advertiseService("/start_bool", &StateMachine::start_bool_callback, this);
        ROS_INFO("server ready, mpc time step T = %.3f", T);
        std::cout << "state machine initialized" << std::endl;
    }
    ~StateMachine() {
        // utils.stop_car();
    }
    ros::NodeHandle& nh;

// private:
    std::vector<Eigen::Vector2d> PARKING_SPOTS;

    // std::array<int, 9> maneuver_indices = {2, 2, 2, 2, 2, 2, 2, 2, 2};
    std::array<int, 9> maneuver_indices = {1,0,2,0,1,2,0,0,0};
    int maneuver_index = 0;
    std::array<double, 4> bbox = {0.0, 0.0, 0.0, 0.0};
    double T_park, T;
    double detected_dist = 0;
    bool right_park = true;
    int park_count = 0;
    int stopsign_flag = 0;
    int maneuver_direction = 0; // 0: left, 1: straight, 2: right
    ros::Time cd_timer = ros::Time::now();
    int end_index = 0;
    Eigen::Vector2d destination;
    Eigen::VectorXd xs;
    int state = 0;
    bool debug = true, sign, ekf, lane, real, dashboard, keyboardControl, hasGps;
    std::string gaz_bool = "_gazebo_";

    ros::Time cooldown_timer;
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
        double x, y, yaw;
        utils.get_states(x, y, yaw);
        mpc.initialize_current_states(x, y, utils.yaw);
        mpc.target_waypoint_index = mpc.find_next_waypoint(mpc.x_current, 0, static_cast<int>(mpc.state_refs.rows() - 1));
        mpc.reset_solver();
        if(hasGps) {
            if(!utils.reinitialize_states()) ROS_WARN("Failed to reinitialize");
        }
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
    void solve();
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
    int change_lane_hardcode(bool right, double rate_val = 20) {
        Eigen::VectorXd targets(1);
        targets << 0.15 * M_PI;
        Eigen::VectorXd steering(1);
        steering << -23.;
        if (right) {
            steering *= -1;
        }
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
            return utils.stopline;
        }
        double lookahead_dist = 0.15;
        int num_index = static_cast<int>(lookahead_dist * mpc.density);
        for (int i = 0; i < num_index; i++) {
            auto attribute_i = mpc.state_attributes(mpc.target_waypoint_index+i); // check 3 waypoints ahead
            if(attribute_i == mpc.ATTRIBUTE::STOPLINE) {
                std::cout << "stopline detected at (" << mpc.state_refs(mpc.target_waypoint_index, 0) << ", " << mpc.state_refs(mpc.target_waypoint_index, 1) << ")" << std::endl;
                double x, y, yaw;
                utils.get_states(x, y, yaw);
                double dist_sq = std::pow(x - last_intersection_point(0), 2) + std::pow(y - last_intersection_point(1), 2);
                if (dist_sq < INTERSECTION_DISTANCE_THRESHOLD * INTERSECTION_DISTANCE_THRESHOLD) {
                    ROS_INFO("intersection detected, but too close to previous intersection: %.3f, ignoring...", std::sqrt(dist_sq));
                    return false;
                }
                last_intersection_point = {x, y};
                return true;
                // if(ros::Time::now() > cd_timer) { // if cooldown timer has expired
                //     cd_timer = ros::Time::now()+ros::Duration(STOP_DURATION*1.5);
                //     if (sign) { // if sign detection is enabled, check for stopsign, otherwise just stop
                //         if (stopsign_flag) {
                //             stopsign_flag = 0;
                //             return true;
                //         }
                //     } else { 
                //         return true;
                //     }
                // } else {
                //     std::cout << "cooldown: " << (cd_timer - ros::Time::now()).toSec() << std::endl;
                //     return false;
                // }
            }
        }
        return false;
    }
    void check_stop_sign() {
        if (!stopsign_flag) {
            int stopsign_index = utils.object_index(OBJECT::STOPSIGN);
            if(stopsign_index >= 0) {
                double dist = utils.object_distance(stopsign_index);
                if (dist < MAX_SIGN_DIST && dist > 0) {
                    std::cout << "stop sign detected at a distance of: " << dist << std::endl;
                    detected_dist = dist;
                    // if(lane) utils.reset_odom();
                    stopsign_flag = STOPSIGN_FLAGS::STOP;
                }
            }
            int light_index = utils.object_index(OBJECT::LIGHTS);
            if(light_index >= 0) {
                double dist = utils.object_distance(light_index);
                if (dist < MAX_SIGN_DIST && dist > 0) {
                    std::cout << "traffic light detected at a distance of: " << dist << std::endl;
                    detected_dist = dist;
                    // if(lane) utils.reset_odom();
                    stopsign_flag = STOPSIGN_FLAGS::LIGHT;
                }
            }
            int prio_index = utils.object_index(OBJECT::PRIORITY);
            if(prio_index >= 0) {
                double dist = utils.object_distance(prio_index);
                if (dist < MAX_SIGN_DIST && dist > 0) {
                    std::cout << "prio detected at a distance of: " << dist << std::endl;
                    detected_dist = dist;
                    stopsign_flag = STOPSIGN_FLAGS::PRIO;
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
    bool park_sign_detected() {
        int park_index = utils.object_index(OBJECT::PARK);
        if(park_index >= 0) {
            double dist = utils.object_distance(park_index);
            if (dist < MAX_PARK_DIST && dist > 0) {
                // std::cout << "parking sign detected at a distance of: " << dist << std::endl;
                detected_dist = dist;
                return true;
            }
        }
        return false;
    }
    double crosswalk_detected() {
        int crosswalk_index = utils.object_index(OBJECT::CROSSWALK);
        if(crosswalk_index >= 0) {
            double dist = utils.object_distance(crosswalk_index);
            if (dist < MAX_CROSSWALK_DIST && dist > 0) {
                std::cout << "crosswalk detected at a distance of: " << dist << std::endl;
                detected_dist = dist;
                return dist;
            }
        }
        return -1;
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
    void lane_follow(double speed = 0.175) {
        double steer = utils.get_steering_angle();
        if(ros::Time::now() < cooldown_timer) {
            speed = speed * 5/7;
        }
        //std::cout << "lanefollowing " << steer<<" " << speed << std::endl;
        utils.publish_cmd_vel(steer, speed);
    }
    void orientation_follow(double orientation, double speed = 0.175) {
        double yaw_error = orientation - utils.get_yaw();
        if(yaw_error > M_PI * 1.5) yaw_error -= 2 * M_PI;
        else if(yaw_error < -M_PI * 1.5) yaw_error += 2 * M_PI;
        double steer = - yaw_error * 180 / M_PI * 1;
        utils.publish_cmd_vel(steer, speed);
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
void StateMachine::solve() {
    // auto start = std::chrono::high_resolution_clock::now();

    int status = mpc.update_and_solve(mpc.x_current);
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

    static bool publish_waypoints = true;
    if(publish_waypoints) {
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
    while (ros::ok()) {
        if (sign) {
            pedestrian_detected();
            exit_detected();
        }
        // if (mpc.target_waypoint_index >= mpc.num_waypoints -1) change_state(STATE::DONE);
        if (state == STATE::MOVING) {
            if(intersection_reached()) {
                if(sign) {
                    if (stopsign_flag == STOPSIGN_FLAGS::STOP || stopsign_flag == STOPSIGN_FLAGS::LIGHT) {
                        stopsign_flag = 0;
                        change_state(STATE::WAITING_FOR_STOPSIGN);
                        continue;
                    } else if (stopsign_flag == STOPSIGN_FLAGS::PRIO) {
                        stopsign_flag = 0;
                        ROS_INFO("priority sign detected, keep moving...");
                    }
                } else {
                    change_state(STATE::WAITING_FOR_STOPSIGN);
                    continue;
                }
            }
            if (sign) {
                check_stop_sign();
                if(park_sign_detected() && park_count < 1) {
                    park_count++;
                    change_state(STATE::PARKING);
                    continue;
                }
                double dist;
                std::list<int> cars = utils.recent_car_indices;
                // int car_index = utils.object_index(OBJECT::CAR);
                // if(car_index >= 0) { // if car detected
                for (int car_index: cars) {
                    dist = utils.object_distance(car_index); // compute distance to back of car
                    if (dist < MAX_CAR_DIST && dist > 0 && mpc.closest_waypoint_index >= end_index * 1.2) {
                        // utils.object_box(car_index, bbox);
                        double x, y, yaw;
                        utils.get_states(x, y, yaw);
                        // auto car_pose = utils.estimate_object_pose2d(x, y, yaw, bbox, dist, CAMERA_PARAMS);
                        auto car_pose = utils.detected_cars[car_index];
                        // printf("estimated car pose: %.3f, %.3f\n", car_pose[0], car_pose[1]);
                        // compute distance from detected car to closest waypoint in front of car to assess whether car is in same lane
                        double look_ahead_dist = dist * 1.5;
                        int look_ahead_index = look_ahead_dist * mpc.density + mpc.closest_waypoint_index;
                        // compute distance from car_pose to waypoint, find closest waypoint and distance
                        double min_dist_sq = 1000.;
                        int min_index = 0;
                        for (int i = mpc.closest_waypoint_index; i < look_ahead_index; i++) {
                            // double dist_sq = (car_pose.head(2) - mpc.state_refs.row(i).head(2)).squaredNorm();
                            double dist_sq = std::pow(car_pose[0] - mpc.state_refs(i, 0), 2) + std::pow(car_pose[1] - mpc.state_refs(i, 1), 2);
                            if (dist_sq < min_dist_sq) {
                                min_dist_sq = dist_sq;
                                min_index = i;
                            }
                        }
                        double min_dist = std::sqrt(min_dist_sq);
                        bool same_lane = false;
                        if (min_dist < LANE_OFFSET * 0.25) {
                            // std::cout << "closest index: " << min_index << ", closest dist: " << min_dist << ", closest waypoint: (" << mpc.state_refs(min_index, 0) << ", " << mpc.state_refs(min_index, 1) << ")" << std::endl;
                            same_lane = true;
                        }
                        // std::cout << "min dist between car and closest waypoint: " << min_dist << ", same lane: " << same_lane << std::endl;
                        if (same_lane) {
                            int idx = mpc.closest_waypoint_index + dist * mpc.density * 0.75; // compute index of midpoint between detected car and ego car
                            int attribute = mpc.state_attributes(idx);
                            // std::cout << "attribute: " << attribute << std::endl;
                            if (attribute != mpc.ATTRIBUTE::DOTTED && attribute != mpc.ATTRIBUTE::DOTTED_CROSSWALK && attribute != mpc.ATTRIBUTE::HIGHWAYLEFT && attribute != mpc.ATTRIBUTE::HIGHWAYRIGHT) {
                                if (dist < MAX_TAILING_DIST) {
                                    ROS_INFO("detected car is in oneway or non-dotted region, dist = %.3f, stopping...", dist);
                                    stop_for(20*T);
                                    continue;
                                } else {
                                    ROS_INFO("car on oneway pretty far and within safety margin, keep tailing: %.3f", dist);
                                }
                            } else {
                                double start_dist = std::max(dist - CAM_TO_CAR_FRONT, MIN_DIST_TO_CAR) - MIN_DIST_TO_CAR;
                                bool right = false;
                                double density = mpc.density;
                                double lane_offset = LANE_OFFSET * 1.5;
                                int end_index_scaler = 1.2;
                                if (attribute == mpc.ATTRIBUTE::HIGHWAYRIGHT) { // if on right side of highway, overtake on left
                                    density *= 1/1.33;
                                    end_index_scaler *= 1.5;
                                    ROS_INFO("in highway right, overtake on left");
                                }
                                else if (attribute == mpc.ATTRIBUTE::HIGHWAYLEFT) { // if on left side of highway, overtake on right
                                    right = true; 
                                    density *= 1/1.33;
                                    end_index_scaler *= 1.5;
                                    ROS_INFO("in highway left, overtake on right");
                                }
                                int start_index = mpc.target_waypoint_index + static_cast<int>(start_dist * density);
                                end_index = start_index + static_cast<int>((CAR_LENGTH * 2 + MIN_DIST_TO_CAR * 2) * density * end_index_scaler);
                                
                                mpc.change_lane(start_index, end_index, right, lane_offset);
                                ROS_INFO("changing lane to the %s in %.3f meters. start pose: (%.2f,%.2f), end: (%.2f, %.2f), cur: (%.2f, %.2f)", start_dist, right ? "right" : "left", mpc.state_refs(start_index, 0), mpc.state_refs(start_index, 1), mpc.state_refs(end_index, 0), mpc.state_refs(end_index, 1), x, y);
                                // stop_for(2*T);
                                // exit(0);
                            }
                        } else {
                            ROS_INFO("detected car is not in the same lane, ignoring...");
                        }
                    }
                }
            }
            update_mpc_state();
            double error_sq = (mpc.x_current.head(2) - destination).squaredNorm();
            if (error_sq < TOLERANCE_SQUARED) {
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
                    update_mpc_state();
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
            static bool use_stopline = false;
            // if (wrong_lane && ros::Time::now() > overtake_cd && (maneuver_index < 2)) {
            //     auto nearest = mpc.NearestDirection(utils.get_yaw());
            //     double yaw_error = nearest - utils.get_yaw();
            //     if(yaw_error < 0.1 * M_PI) {
            //         ROS_INFO("overtake timer expired. changing back to original lane");
            //         wrong_lane = false;
            //         change_lane_hardcode(true);
            //     }
            // }
            // if(cooldown_timer < ros::Time::now() && intersection_reached(true)) {
            if(cooldown_timer < ros::Time::now()) {
                if (use_stopline) {
                    if(intersection_reached(true)) {
                        ROS_INFO("used stopling to detect intersection");
                        if(stopsign_flag == STOPSIGN_FLAGS::STOP || stopsign_flag == STOPSIGN_FLAGS::LIGHT) {
                            stopsign_flag = STOPSIGN_FLAGS::NONE;
                            ROS_INFO("stop sign detected");
                            change_state(STATE::WAITING_FOR_STOPSIGN);
                            continue;
                        } else {
                            ROS_INFO("no stop sign or light detected");
                            change_state(STATE::INTERSECTION_MANEUVERING);
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
            }
            if (sign && cooldown_timer < ros::Time::now()) {
                double crosswalk_dist = crosswalk_detected();
                if(crosswalk_dist > 0) {
                    double cd = (crosswalk_dist + CROSSWALK_LENGTH) / 0.15;
                    cooldown_timer = ros::Time::now() + ros::Duration(cd);
                    ROS_INFO("crosswalk detected at a distance of: %.3f, slowing down for %.3f", crosswalk_dist, cd);
                    continue;
                }
                check_stop_sign();
                if(park_sign_detected() && park_count < 1) {
                    park_count++;
                    change_state(STATE::PARKING);
                    continue;
                }
                double dist;
                int car_index = utils.object_index(OBJECT::CAR);
                if(car_index >= 0 && ros::Time::now() > overtake_cd) { // if car detected
                    ROS_INFO("car detected");
                    dist = utils.object_distance(car_index); // compute distance to back of car
                    if (dist < MAX_CAR_DIST && dist > 0) {
                        std::cout << "a car is detected at a distance of " << dist << std::endl;
                        // std::array<double, 4> car_bbox = utils.object_box(car_index);
                        // utils.object_box(car_index, bbox);
                        // double x, y, yaw;
                        // utils.get_states(x, y, yaw);
                        // auto car_pose = utils.estimate_object_pose2d(0, 0, yaw, bbox, dist, CAMERA_PARAMS);
                        // printf("estimated relative car pose: %.3f, %.3f\n", car_pose[0], car_pose[1]);
                        // compute distance from detected car to closest waypoint in front of car to assess whether car is in same lane
                        bool same_lane = true;
                        if (same_lane) {
                            static bool dotted = true;
                            if (maneuver_index > 2)  dotted = false;
                            if (!dotted) {
                                if (dist < MAX_TAILING_DIST) {
                                    std::cout << "detected car is in oneway or non-dotted region, dist =" << dist << std::endl;
                                    stop_for(T);
                                    continue;
                                } else {
                                    std::cout << "car on oneway pretty far and within safety margin, keep tailing: " << dist << std::endl;
                                }
                            } else {
                                // dotted = false;
                                double safe_dist = std::max(dist + CAM_TO_CAR_FRONT, MIN_DIST_TO_CAR) - MIN_DIST_TO_CAR;
                                overtake_cd = ros::Time::now() + ros::Duration((dist + 1.0) / NORMAL_SPEED);
                                bool right = false;
                                wrong_lane = true;
                                std::cout << wrong_lane << ", " << (overtake_cd - ros::Time::now()).toSec() << std::endl;
                                ROS_INFO("detected car at a distance of: %.3f, changing lane, right: %s", dist, right ? "true" : "false");
                                ROS_INFO("overtake cooldown: %.3f", (dist + 0.35) / NORMAL_SPEED);
                                change_lane_hardcode(right);
                            }
                        }
                    }
                }
            }
            update_mpc_state();
            // double error_sq = (mpc.x_current.head(2) - destination).squaredNorm();
            // if (error_sq < TOLERANCE_SQUARED) {
            //     change_state(STATE::DONE);
            // }
            if(intersection_reached(true)) {
                double orientation = mpc.NearestDirection(utils.get_yaw());
                orientation_follow(orientation);
            } else {
                lane_follow();
                // double orientation = mpc.NearestDirection(utils.get_yaw());
                // orientation_follow(orientation);
            }
            rate->sleep();
            continue;
        } else if (state == STATE::WAITING_FOR_STOPSIGN) {
            stop_for(STOP_DURATION);
            cooldown_timer = ros::Time::now() + ros::Duration(SIGN_COOLDOWN);
            if (lane) {
                change_state(STATE::INTERSECTION_MANEUVERING);
            } else {
                change_state(STATE::MOVING);
            }
        } else if (state == STATE::WAITING_FOR_LIGHT) {
            stop_for(STOP_DURATION);
            cooldown_timer = ros::Time::now() + ros::Duration(SIGN_COOLDOWN);
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
                // xs << base_offset, 0., 0., 0., 0.;
                // ROS_INFO("moving to: %.3f, %.3f, %.3f", xs[0], xs[1], xs[2]);
                // ros::Rate temp_rate(1/T_park);
                // std::cout << "park rate: " << 1/T_park << std::endl;
                // mpc.set_up_park(xs);
                // int status, i = 0;
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
            ROS_INFO("maneuvering");
            maneuver_direction = maneuver_indices[maneuver_index++];
            bool use_mpc_for_maneuvering = false;
            ROS_INFO("direction: %s", MANEUVER_DIRECTIONS[maneuver_direction].c_str());
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
                    if (maneuver_direction == 1) { // if straight
                        double distance_traveled = std::sqrt(std::pow(x - x0, 2) + std::pow(y - y0, 2));
                        if (distance_traveled > 0.57) {
                            break;
                        }
                    } else {
                        double yaw_error = target_yaw - utils.get_yaw();
                        if (std::abs(yaw_error) < 0.325) {
                            break;
                        }
                    }
                    odomFrame << x - x0, y - y0;
                    // transformedFrame = rotation_matrices[direction_index] * odomFrame;
                    transformedFrame = odomFrame.transpose() * rotation_matrices[direction_index];
                    double referenceY = utils.computeTrajectory(transformedFrame[0]);
                    double error = (referenceY - transformedFrame[1]);
                    double steer = -utils.computeTrajectoryPid(error);
                    // ROS_INFO("target_yaw: %2f, cur_yaw: %2f, yaw_err: %.3f, tar y: %.3f, cur y: %.3f, err: %.3f, steer: %.3f", target_yaw, utils.get_yaw(), yaw_error, referenceY, transformedFrame[1], error, steer);
                    // ROS_INFO("x:%.3f, trans x:%.3f, tar y:%.3f, cur y:%.3f, err:%.3f, steer:%.3f", odomFrame[0], transformedFrame[0], referenceY, transformedFrame[1], error, steer);
                    utils.publish_cmd_vel(steer, 0.25);
                    rate->sleep();
                }
            }
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
    // while(!sm.utils.initializationFlag) {
    //     ros::Duration(0.1).sleep();
    // }
    
    sm.run();

    // t.join();
    t2.join();
    std::cout << "threads joined" << std::endl;
    return 0;
}
