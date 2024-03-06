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

class StateMachine {
public:
    StateMachine(ros::NodeHandle& nh_, double T, int N, double v_ref, bool sign, bool ekf, bool lane, double T_park, std::string robot_name, double x_init, double y_init, double yaw_init): 
    nh(nh_), utils(nh, x_init, y_init, yaw_init, sign, ekf, lane, robot_name), mpc(T,N,v_ref), cooldown_timer(ros::Time::now()), xs(5),
    state(STATE::INIT), sign(sign), ekf(ekf), lane(lane), T_park(T_park), T(T), detected_index(0)
    {
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
        // ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
        while(!utils.initializationFlag) {
            ros::spinOnce();
            rate->sleep();
        }
        double x, y, yaw;
        utils.get_states(x, y, yaw);
        std::cout << "initialized: " << utils.initializationFlag << ", gps_x: " << x << ", gps_y: " << y << ", yaw:" << utils.yaw << std::endl;
        if (ekf) std::cout << "ekf_x: " << utils.ekf_x << ", ekf_y: " << utils.ekf_y << ", ekf_yaw:" << utils.ekf_yaw << std::endl;
        mpc.update_current_states(x, y, utils.yaw);
        mpc.target_waypoint_index = mpc.find_next_waypoint(mpc.x_current, 0, static_cast<int>(mpc.state_refs.rows() - 1));
    }
    ~StateMachine() {
        // utils.stop_car();
    }
    ros::NodeHandle& nh;

// private:
    // Constants
    const std::array<std::string, 11> state_names = {
        "INIT", "MOVING", "APPROACHING_INTERSECTION", "WAITING_FOR_STOPSIGN",
        "WAITING_FOR_LIGHT", "PARKING", "PARKED", "EXITING_PARKING", "DONE", "LANE_FOLLOWING", "INTERSECTION_MANEUVERING"
    };
    enum STATE {
        INIT,
        MOVING,
        APPROACHING_INTERSECTION,
        WAITING_FOR_STOPSIGN,
        WAITING_FOR_LIGHT,
        PARKING,
        PARKED,
        EXITING_PARKING,
        DONE,
        LANE_FOLLOWING,
        INTERSECTION_MANEUVERING
    };
    enum STOPSIGN_FLAGS {
        NONE,
        STOP,
        LIGHT,
        PRIO
    };
    enum OBJECT {
        ONEWAY,
        HIGHWAYENTRANCE,
        STOPSIGN,
        ROUNDABOUT,
        PARK,
        CROSSWALK,
        NOENTRY,
        HIGHWAYEXIT,
        PRIORITY,
        LIGHTS,
        BLOCK,
        PEDESTRIAN,
        CAR,
    };
    
    static constexpr double CAM_TO_CAR_FRONT = 0.21;
    static constexpr double CAR_LENGTH = 0.464;
    static constexpr double MAX_TAILING_DIST = 1.0;
    static constexpr double MIN_SIGN_DIST = 0.39;  // 0.6 - 0.21
    // static constexpr double MAX_SIGN_DIST = 1.09;
    static constexpr double MAX_SIGN_DIST = 0.753;
    static constexpr double MAX_PARK_DIST = 0.79;  // 1.0 - 0.21
    static constexpr double MAX_CROSSWALK_DIST = 0.79;  // 1.0 - 0.21
    static constexpr double PARKSIGN_TO_CAR = 0.51;
    static constexpr double PARK_OFFSET = 1.31;    // 1.1 + 0.21
    static constexpr double PARKING_SPOT_LENGTH = 0.723;
    static constexpr double CROSSWALK_LENGTH = 1.0;
    static constexpr double OVERTAKE_DIST = 2.0;
    static constexpr double LANE_OFFSET = 0.36;
    static constexpr double MIN_DIST_TO_CAR = 0.8;
    static constexpr double MAX_CAR_DIST = 3.0;
    static constexpr double SIGN_COOLDOWN = 1.0;
    static constexpr double TOLERANCE_SQUARED = 0.01;
    static constexpr double STOP_DURATION = 1.50;
    static constexpr double NORMAL_SPEED = 0.175;
    static constexpr double FAST_SPEED = 0.4;
    std::array<double, 2> PARKING_SPOT_RIGHT = {9.50, 0.372};
    std::array<double, 2> PARKING_SPOT_LEFT = {9.50, 1.110};
    std::vector<Eigen::Vector2d> PARKING_SPOTS;

    // std::array<int, 9> maneuver_indices = {2, 2, 2, 2, 2, 2, 2, 2, 2};
    std::array<int, 9> maneuver_indices = {0,2,1,0,1,2,0,0,0};
    int maneuver_index = 0;
    std::array<double, 4> bbox = {0.0, 0.0, 0.0, 0.0};
    double T_park, T;
    double detected_dist = 0;
    bool right_park = true;
    int park_count = 0;
    int stopsign_flag = 0;
    int maneuver_direction = 0; // 0: left, 1: straight, 2: right
    const std::array<std::string, 3> MANEUVER_DIRECTIONS = {"left", "straight", "right"};
    ros::Time cd_timer = ros::Time::now();
    int detected_index = 0;
    Eigen::Vector2d destination;
    Eigen::VectorXd xs;
    int state = 0;
    bool debug = true, sign, ekf, lane;
    std::string gaz_bool = "_gazebo_";

    ros::Time cooldown_timer;
    std::array<double, 3> x0;
    ros::Rate* rate;

    std::mutex lock;
    Utility utils;
    Optimizer mpc;

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
    int parking_maneuver_hardcode(bool right = true, bool exit = false, double rate_val = 10);
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
        if(lane) {
            return utils.stopline;
        }
        auto attribute3 = mpc.state_attributes(mpc.target_waypoint_index+3); // check 3 waypoints ahead
        if(attribute3 == mpc.ATTRIBUTE::STOPLINE) {
            std::cout << "stopline detected at (" << mpc.state_refs(mpc.target_waypoint_index, 0) << ", " << mpc.state_refs(mpc.target_waypoint_index, 1) << ")" << std::endl;
            if(ros::Time::now() > cd_timer) { // if cooldown timer has expired
                cd_timer = ros::Time::now()+ros::Duration(STOP_DURATION*1.5);
                if (sign) { // if sign detection is enabled, check for stopsign, otherwise just stop
                    if (stopsign_flag) {
                        stopsign_flag = 0;
                        return true;
                    }
                } else { 
                    return true;
                }
            } else {
                std::cout << "cooldown: " << (cd_timer - ros::Time::now()).toSec() << std::endl;
                return false;
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
                    utils.reset_odom();
                    stopsign_flag = STOPSIGN_FLAGS::STOP;
                }
            }
            int light_index = utils.object_index(OBJECT::LIGHTS);
            if(light_index >= 0) {
                double dist = utils.object_distance(light_index);
                if (dist < MAX_SIGN_DIST && dist > 0) {
                    std::cout << "traffic light detected at a distance of: " << dist << std::endl;
                    detected_dist = dist;
                    utils.reset_odom();
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
        //         // auto world_pose = utils.estimate_object_pose2d(utils.get_real_states(), box, dist, utils.CAMERA_PARAMS);
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
        //         // auto world_pose = utils.estimate_object_pose2d(utils.get_real_states(), box, dist, utils.CAMERA_PARAMS);
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
                std::cout << "parking sign detected at a distance of: " << dist << std::endl;
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
        if (utils.object_index(OBJECT::PEDESTRIAN) >= 0 || utils.object_index(OBJECT::HIGHWAYEXIT) >= 0) {
            while (true) {
                if (utils.object_index(OBJECT::PEDESTRIAN) >= 0 || utils.object_index(OBJECT::HIGHWAYEXIT) >= 0) {
                    double dist = utils.object_distance(utils.object_index(OBJECT::HIGHWAYEXIT));
                    ROS_INFO("girl detected at a distance of: %3f", dist);
                    stop_for(STOP_DURATION);
                } else {
                    pedestrian_count ++;
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

int StateMachine::parking_maneuver_hardcode(bool right, bool exit, double rate_val) {
    double x0, y0, yaw0;
    // get current states
    utils.get_states(x0, y0, yaw0);
    // get closest direction
    yaw0 = mpc.NearestDirection(yaw0);
    while (yaw0 < -M_PI) yaw0 += 2*M_PI;
    while (yaw0 > M_PI) yaw0 -= 2*M_PI;
    ROS_INFO("nearest direction: %.3f", yaw0);
    int stage = 1;
    double steering_angle = -23.;
    double speed = -0.2;
    double thresh = 0.1;
    if (right) {
        steering_angle *= -1;
    }
    if (exit) {
        speed *= -1;
        steering_angle *= -1;
        thresh *= 2;
    }
    printf("yaw_0: %3f, steering_angle: %3f, speed: %3f, park right: %s\n", yaw0, steering_angle, speed, right ? "true" : "false");

    ros::Rate temp_rate(rate_val);

    while(1) {
        double yaw = utils.get_yaw();
        while (yaw < -M_PI) yaw += 2*M_PI;
        while (yaw > M_PI) yaw -= 2*M_PI;
        double yaw_error = yaw - yaw0;
        while(std::abs(yaw_error) > M_PI * 1.2) {
            if(yaw_error > M_PI * 1.2) {
                yaw_error -= 2*M_PI;
            } else {
                yaw_error += 2*M_PI;
            }
        }
        ROS_INFO("yaw: %3f, yaw0: %.3f, yaw_error: %3f, steering_angle: %3f", yaw, yaw0, yaw_error, steering_angle);
        if (stage == 1 && std::abs(yaw_error) > M_PI*0.18) {
            ROS_INFO("stage 1 completed. yaw error: %3f", yaw_error);
            stage = 2;
            steering_angle *= -1;
            continue;
        } else if (stage == 2 && std::abs(yaw_error) < thresh) {
            ROS_INFO("stage 2 completed. yaw error: %3f", yaw_error);
            if(exit) break;
            speed = std::abs(speed);
            steering_angle *= -1;
            thresh /= 3;
            stage = 3;
        } else if (stage == 3 && std::abs(yaw_error) < thresh) {
            ROS_INFO("stage 3 completed. yaw error: %3f", yaw_error);
            break;
        }
        utils.publish_cmd_vel(steering_angle, speed);
        temp_rate.sleep();
    }
    return 0;
}
void StateMachine::update_mpc_state() {
    double x, y, yaw;
    // if (ekf) {
    //     utils.get_ekf_states(x, y, yaw);
    //     mpc.update_current_states(utils.gps_x, utils.gps_y, utils.yaw, mpc.x_real); // update real states as well
    // } else {
    //     utils.get_gps_states(x, y, yaw);
    // }
    utils.get_states(x, y, yaw);
    mpc.update_current_states(x, y, yaw); // no argument means use current states
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
    double steer = -mpc.u_current[1] * 180 / M_PI; // convert to degrees
    double speed = mpc.u_current[0];
    // steer = 23;
    // speed = 0.573;
    utils.publish_cmd_vel(steer, speed);
}
void StateMachine::change_state(STATE new_state) {
    std::cout << "Changing from " << state_names[state] << " to " << state_names[new_state] << std::endl;
    state = new_state;
}
void StateMachine::run() {
    while (ros::ok()) {
        if (sign) {
            pedestrian_detected();
            exit_detected();
        }
        if (mpc.target_waypoint_index >= mpc.num_waypoints -1) change_state(STATE::DONE);
        if (state == STATE::MOVING) {
            if(intersection_reached()) {
                change_state(STATE::WAITING_FOR_STOPSIGN);
                continue;
            }
            // auto attribute3 = mpc.state_attributes(mpc.target_waypoint_index+3); // check 3 waypoints ahead
            // if(attribute3 == mpc.ATTRIBUTE::STOPLINE) {
            //     std::cout << "stopline detected at (" << mpc.state_refs(mpc.target_waypoint_index, 0) << ", " << mpc.state_refs(mpc.target_waypoint_index, 1) << ")" << std::endl;
            //     if(ros::Time::now() > cd_timer) { // if cooldown timer has expired
            //         cd_timer = ros::Time::now()+ros::Duration(STOP_DURATION*1.5);
            //         if (sign) { // if sign detection is enabled, check for stopsign, otherwise just stop
            //             if (stopsign_flag) {
            //                 stopsign_flag = false;
            //                 change_state(STATE::WAITING_FOR_LIGHT);
            //                 continue;
            //             }
            //         } else { 
            //             change_state(STATE::WAITING_FOR_LIGHT);
            //             continue;
            //         }
            //     } else {
            //         std::cout << "cooldown: " << (cd_timer - ros::Time::now()).toSec() << std::endl;
            //     }
            // }
            if (sign) {
                check_stop_sign();
                if(park_sign_detected() && park_count < 1) {
                    park_count++;
                    change_state(STATE::PARKING);
                    continue;
                }
                double dist;
                int car_index = utils.object_index(OBJECT::CAR);
                if(car_index >= 0) { // if car detected
                    dist = utils.object_distance(car_index); // compute distance to back of car
                    if (dist < MAX_CAR_DIST && dist > 0 && mpc.closest_waypoint_index >= detected_index * 1.2) {
                        std::cout << "a car is detected at a distance of " << dist << ", at index: " << mpc.closest_waypoint_index << ", detected_index: " << detected_index << std::endl;
                        // std::array<double, 4> car_bbox = utils.object_box(car_index);
                        utils.object_box(car_index, bbox);
                        double x, y, yaw;
                        utils.get_states(x, y, yaw);
                        auto car_pose = utils.estimate_object_pose2d(x, y, yaw, bbox, dist, utils.CAMERA_PARAMS);
                        printf("estimated car pose: %3f, %3f\n", car_pose[0], car_pose[1]);
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
                            std::cout << "closest index: " << min_index << ", closest dist: " << min_dist << ", closest waypoint: (" << mpc.state_refs(min_index, 0) << ", " << mpc.state_refs(min_index, 1) << ")" << std::endl;
                            same_lane = true;
                        }
                        std::cout << "min dist between car and closest waypoint: " << min_dist << ", same lane: " << same_lane << std::endl;
                        if (same_lane) {
                            int idx = mpc.closest_waypoint_index + dist * mpc.density * 0.75; // compute index of midpoint between detected car and ego car
                            int attribute = mpc.state_attributes(idx);
                            std::cout << "attribute: " << attribute << std::endl;
                            if (attribute != mpc.ATTRIBUTE::DOTTED && attribute != mpc.ATTRIBUTE::DOTTED_CROSSWALK && attribute != mpc.ATTRIBUTE::HIGHWAYLEFT && attribute != mpc.ATTRIBUTE::HIGHWAYRIGHT) {
                                if (dist < MAX_TAILING_DIST) {
                                    std::cout << "detected car is in oneway or non-dotted region, dist =" << dist << std::endl;
                                    stop_for(T);
                                    continue;
                                } else {
                                    std::cout << "car on oneway pretty far and within safety margin, keep tailing: " << dist << std::endl;
                                }
                            } else {
                                dist = std::max(dist + CAM_TO_CAR_FRONT, MIN_DIST_TO_CAR) - MIN_DIST_TO_CAR;
                                bool right = false;
                                double density = mpc.density;
                                if (attribute == mpc.ATTRIBUTE::HIGHWAYRIGHT) { // if on right side of highway, overtake on left
                                    density *= 1/1.33;
                                }
                                else if (attribute == mpc.ATTRIBUTE::HIGHWAYLEFT) { // if on left side of highway, overtake on right
                                    right = true; 
                                    density *= 1/1.33;
                                }
                                int offset = static_cast<int>(dist * density);
                                detected_index = mpc.target_waypoint_index + static_cast<int>(OVERTAKE_DIST * density);
                                std::cout << "detected car at a distance of: " << dist << ", changing lane, right: " << right << ", current index: " << mpc.target_waypoint_index << ", detected index: " << detected_index << ", offset: " << offset << std::endl;
                                mpc.change_lane(mpc.target_waypoint_index+offset, detected_index+offset, right, LANE_OFFSET);
                            }
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
            double offset = std::max(0.0, detected_dist - MIN_SIGN_DIST) * 0.7;
            utils.reset_odom();
            double orientation = mpc.NearestDirection(utils.get_yaw());
            ROS_INFO("sign detected at a distance of: %3f, offset: %3f, resetting odom...", detected_dist, offset);
            while(1) {
                double norm = utils.odomX * utils.odomX + utils.odomY * utils.odomY;
                // ROS_INFO("norm: %3f", norm);
                if (norm >= offset)
                {
                    ROS_INFO("intersection reached, stopping...");
                    utils.publish_cmd_vel(0.0, 0.0);
                    break;
                }
                if (lane) {
                    if (norm > offset*0.2) {
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
                    ROS_INFO("crosswalk detected at a distance of: %3f, slowing down for %3f", crosswalk_dist, cd);
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
                if(car_index >= 0) { // if car detected
                    dist = utils.object_distance(car_index); // compute distance to back of car
                    if (dist < MAX_CAR_DIST && dist > 0) {
                        std::cout << "a car is detected at a distance of " << dist << std::endl;
                        // std::array<double, 4> car_bbox = utils.object_box(car_index);
                        utils.object_box(car_index, bbox);
                        double x, y, yaw;
                        utils.get_states(x, y, yaw);
                        auto car_pose = utils.estimate_object_pose2d(x, y, yaw, bbox, dist, utils.CAMERA_PARAMS);
                        printf("estimated car pose: %3f, %3f\n", car_pose[0], car_pose[1]);
                        // compute distance from detected car to closest waypoint in front of car to assess whether car is in same lane
                        bool same_lane = true;
                        if (same_lane) {
                            int idx = mpc.closest_waypoint_index + dist * mpc.density * 0.75; // compute index of midpoint between detected car and ego car
                            int attribute = mpc.state_attributes(idx);
                            std::cout << "attribute: " << attribute << std::endl;
                            if (true) {
                                if (dist < MAX_TAILING_DIST) {
                                    std::cout << "detected car is in oneway or non-dotted region, dist =" << dist << std::endl;
                                    stop_for(T);
                                    continue;
                                } else {
                                    std::cout << "car on oneway pretty far and within safety margin, keep tailing: " << dist << std::endl;
                                }
                            } else {
                                dist = std::max(dist + CAM_TO_CAR_FRONT, MIN_DIST_TO_CAR) - MIN_DIST_TO_CAR;
                                bool right = false;
                                double density = mpc.density;
                                if (attribute == mpc.ATTRIBUTE::HIGHWAYRIGHT) { // if on right side of highway, overtake on left
                                    density *= 1/1.33;
                                }
                                else if (attribute == mpc.ATTRIBUTE::HIGHWAYLEFT) { // if on left side of highway, overtake on right
                                    right = true; 
                                    density *= 1/1.33;
                                }
                                int offset = static_cast<int>(dist * density);
                                detected_index = mpc.target_waypoint_index + static_cast<int>(OVERTAKE_DIST * density);
                                std::cout << "detected car at a distance of: " << dist << ", changing lane, right: " << right << ", current index: " << mpc.target_waypoint_index << ", detected index: " << detected_index << ", offset: " << offset << std::endl;
                                mpc.change_lane(mpc.target_waypoint_index+offset, detected_index+offset, right, LANE_OFFSET);
                            }
                        }
                    }
                }
            }
            update_mpc_state();
            double error_sq = (mpc.x_current.head(2) - destination).squaredNorm();
            if (error_sq < TOLERANCE_SQUARED) {
                change_state(STATE::DONE);
            }
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
            change_state(STATE::MOVING);
            if(lane) change_state(STATE::LANE_FOLLOWING);
        } else if (state == STATE::PARKING) {
            stop_for(STOP_DURATION/2);
            double offset_thresh = 0.1;
            double base_offset = detected_dist + PARKING_SPOT_LENGTH * 1.5 + offset_thresh;
            base_offset *= 1.7; // temporary fix for gazebo
            ROS_INFO("parking offset is: %3f", base_offset);
            // base_offset = 0;
            right_park = true;
            bool hard_code = true;
            if (!lane) {
                xs << base_offset, 0., 0., 0., 0.;
                ROS_INFO("moving to: %3f, %3f, %3f", xs[0], xs[1], xs[2]);
                // move_to(xs);
                ros::Rate temp_rate(1/T_park);
                std::cout << "park rate: " << 1/T_park << std::endl;
                mpc.set_up_park(xs);
                int status, i = 0;
                int target_spot = 0;
                std::cout << "target spot: " << target_spot << std::endl;
                while(1) {
                    pedestrian_detected();
                    exit_detected();
                    std::list<int> cars = utils.recent_car_indices;
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
                            // auto world_pose = utils.estimate_object_pose2d(x, y, yaw, box, dist, utils.CAMERA_PARAMS);
                            Eigen::Vector2d world_pose = utils.detected_cars[i];
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
                        xs[0] = base_offset + target_spot/2 * PARKING_SPOT_LENGTH;
                        right_park = target_spot % 2 == 0;
                        ROS_INFO("car in spot, changing to target spot %d at (%3f, %3f), right: %s", target_spot, PARKING_SPOTS[target_spot][0], PARKING_SPOTS[target_spot][1], right_park ? "true" : "false");
                        mpc.set_up_park(xs);
                    }
                    update_mpc_state();
                    status = mpc.update_and_solve_park(xs, offset_thresh*offset_thresh);
                    publish_commands();
                    if (status == 2 || i>200) {
                        break;
                    }
                    i++;
                    temp_rate.sleep();
                }
            } else {
                double orientation = mpc.NearestDirection(utils.get_yaw());
                ROS_INFO("orientation: %.3f", orientation);
                utils.reset_odom();
                while(1) {
                    pedestrian_detected();
                    exit_detected();
                    double norm = utils.odomX * utils.odomX + utils.odomY * utils.odomY;
                    ROS_INFO("norm: %3f", norm);
                    if (norm >= base_offset)
                    {
                        ROS_INFO("parking spot reached, stopping...");
                        utils.publish_cmd_vel(0.0, 0.0);
                        break;
                    }
                    if (norm > base_offset*0.2) {
                        orientation_follow(orientation);
                    } else {
                        lane_follow();
                    }
                    rate->sleep();
                }
            }
            stop_for(STOP_DURATION/2);
            if (hard_code) {
                right_park = true; //temp
                parking_maneuver_hardcode(right_park, false, 1/T_park);
            } else {
                xs << -0.63, -0.35, 0. , 0., 0.;
                ROS_INFO("moving to: %3f, %3f, %3f", xs[0], xs[1], xs[2]);
                move_to(xs);
            }
            change_state(STATE::PARKED);
        } else if (state == STATE::PARKED) {
            stop_for(STOP_DURATION);
            change_state(STATE::EXITING_PARKING);
        } else if (state == STATE::EXITING_PARKING) {
            bool hard_code = true;
            if (hard_code) {
                right_park = true;
                parking_maneuver_hardcode(right_park, true, 1/T_park);
            } else {
                xs << -0.2, 0., 0. , 0., 0.;
                ROS_INFO("moving back for %3f", xs[0]);
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
                    // ROS_INFO("error_sq: %3f", error_sq);
                    int status = mpc.update_and_solve(mpc.x_current_transformed, maneuver_direction);
                    publish_commands();
                    rate->sleep();
                }
                mpc.last_waypoint_index = last_history;
            } else {
                utils.reset_odom();
                double x0, y0, yaw0;
                yaw0 = mpc.NearestDirection(utils.get_yaw());
                static const double directions[5] = {0, M_PI / 2, M_PI, 3 * M_PI / 2, 2 * M_PI};
                int direction_index = mpc.NearestDirectionIndex(yaw0);
                int target_index = (direction_index - (maneuver_direction-1)) % 4;
                if(target_index < 0) target_index += 4;
                double target_yaw = directions[target_index];
                while(target_yaw > M_PI) target_yaw -= 2*M_PI;
                while(target_yaw < -M_PI) target_yaw += 2*M_PI;
                ROS_INFO("target_yaw: %.3f, cur_yaw: %.3f, direction_index: %d, target_index: %d", target_yaw, yaw0, direction_index, target_index);
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
                    if (maneuver_direction == 1) { // if straight
                        double distance_traveled = std::sqrt(utils.odomX * utils.odomX + utils.odomY * utils.odomY);
                        if (distance_traveled > 0.57) {
                            break;
                        }
                    } else {
                        double yaw_error = target_yaw - utils.get_yaw();
                        if (std::abs(yaw_error) < 0.357) {
                            break;
                        }
                    }
                    odomFrame << utils.odomX, utils.odomY;
                    // transformedFrame = rotation_matrices[direction_index] * odomFrame;
                    transformedFrame = odomFrame.transpose() * rotation_matrices[direction_index];
                    double referenceY = utils.computeTrajectory(transformedFrame[0]);
                    double error = (referenceY - transformedFrame[1]);
                    double steer = -utils.computeTrajectoryPid(error);
                    // ROS_INFO("target_yaw: %2f, cur_yaw: %2f, yaw_err: %3f, tar y: %3f, cur y: %3f, err: %3f, steer: %3f", target_yaw, utils.get_yaw(), yaw_error, referenceY, transformedFrame[1], error, steer);
                    // ROS_INFO("x:%.3f, trans x:%.3f, tar y:%.3f, cur y:%.3f, err:%.3f, steer:%.3f", odomFrame[0], transformedFrame[0], referenceY, transformedFrame[1], error, steer);
                    utils.publish_cmd_vel(steer, 0.25);
                    rate->sleep();
                }
            }
            change_state(STATE::LANE_FOLLOWING);
        } else if (state == STATE::INIT) {
            change_state(STATE::MOVING);
            if (lane) change_state(STATE::LANE_FOLLOWING);
            // change_state(STATE::INTERSECTION_MANEUVERING);
            // change_state(STATE::PARKING);
        } else if (state == STATE::DONE) {
            std::cout << "Done" << std::endl;
            utils.stop_car();
            if (debug) {
                mpc.computeStats(357);
            }
            break;
        }
    }
}


StateMachine *globalStateMachinePtr = nullptr;
void signalHandler(int signum) {
    if (globalStateMachinePtr) {
        globalStateMachinePtr->utils.stop_car();
        //globalStateMachinePtr->mpc.computeStats(357);
        //globalStateMachinePtr->utils.print_detected_cars();
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
    bool sign, ekf, lane;
    std::string name;
    std::string nodeName = ros::this_node::getName();
    std::cout << "node name: " << nodeName << std::endl;
    bool success = nh.getParam(nodeName + "/lane", lane) && nh.getParam(nodeName+"/ekf", ekf) && nh.getParam(nodeName+"/sign", sign) && nh.getParam("T", T) && nh.getParam("N", N) && nh.getParam("constraints/v_ref", v_ref);
    double x0, y0, yaw0, vref;
    success = success && nh.getParam(nodeName+"/name", name) && nh.getParam(nodeName+"/vref", vref) && nh.getParam(nodeName+"/x0", x0) && nh.getParam(nodeName+"/y0", y0) && nh.getParam(nodeName+"/yaw0", yaw0);
    success = success && nh.getParam("/T_park", T_park);
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
    std::cout << "ekf: " << ekf << ", sign: " << sign << ", T: " << T << ", N: " << N << ", vref: " << vref << std::endl;
    StateMachine sm(nh, T, N, vref, sign, ekf, lane, T_park, name, x0, y0, yaw0);

    globalStateMachinePtr = &sm;
    signal(SIGINT, signalHandler);

    // std::thread t(&StateMachine::run, &sm);
    std::thread t2(&Utility::spin, &sm.utils);
    while(!sm.utils.initializationFlag) {
        ros::Duration(0.1).sleep();
    }
    
    sm.run();

    // t.join();
    t2.join();
    std::cout << "threads joined" << std::endl;
    return 0;
}
