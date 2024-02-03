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
    StateMachine(ros::NodeHandle& nh_, double T, int N, double v_ref, bool sign, bool ekf, bool lane, double T_park, std::string robot_name): 
    nh(nh_), utils(nh, sign, ekf, lane, robot_name), mpc(T,N,v_ref), cooldown_timer(ros::Time::now()), xs(5),
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
        ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states");
        while(!utils.initializationFlag) {
            ros::spinOnce();
            rate->sleep();
        }
        double x, y, yaw;
        if (ekf) {
            utils.get_ekf_states(x, y, yaw);
        } else {
            utils.get_gps_states(x, y, yaw);
        }
        std::cout << "initialized: " << utils.initializationFlag << ", gps_x: " << x << ", gps_y: " << y << ", yaw:" << utils.yaw << std::endl;
        if (ekf) std::cout << "ekf_x: " << utils.ekf_x << ", ekf_y: " << utils.ekf_y << ", ekf_yaw:" << utils.ekf_yaw << std::endl;
        mpc.update_current_states(x, y, utils.yaw);
        mpc.target_waypoint_index = mpc.find_next_waypoint(0, static_cast<int>(mpc.state_refs.rows() - 1));
    }
    ~StateMachine() {
        // utils.stop_car();
    }
    ros::NodeHandle& nh;

// private:
    // Constants
    const std::array<std::string, 10> state_names = {
        "INIT", "MOVING", "APPROACHING_INTERSECTION", "WAITING_FOR_STOPSIGN",
        "WAITING_FOR_LIGHT", "PARKING", "PARKED", "EXITING_PARKING", "DONE", "LANE_FOLLOWING"
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
    std::array<double, 2> PARKING_SPOT_RIGHT = {9.50, 0.372};
    std::array<double, 2> PARKING_SPOT_LEFT = {9.50, 1.110};
    std::vector<Eigen::Vector2d> PARKING_SPOTS;

    std::array<double, 4> bbox = {0.0, 0.0, 0.0, 0.0};
    double T_park, T;
    double detected_dist = 0;
    bool right_park = true;
    int park_count = 0;
    bool stopsign_flag = false;
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
    bool intersection_reached() {
        auto attribute3 = mpc.state_attributes(mpc.target_waypoint_index+3); // check 3 waypoints ahead
        if(attribute3 == mpc.ATTRIBUTE::STOPLINE) {
            std::cout << "stopline detected at (" << mpc.state_refs(mpc.target_waypoint_index, 0) << ", " << mpc.state_refs(mpc.target_waypoint_index, 1) << ")" << std::endl;
            if(ros::Time::now() > cd_timer) { // if cooldown timer has expired
                cd_timer = ros::Time::now()+ros::Duration(STOP_DURATION*1.5);
                if (sign) { // if sign detection is enabled, check for stopsign, otherwise just stop
                    if (stopsign_flag) {
                        stopsign_flag = false;
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
                    stopsign_flag = true;
                }
            }
            int light_index = utils.object_index(OBJECT::LIGHTS);
            if(light_index >= 0) {
                double dist = utils.object_distance(light_index);
                if (dist < MAX_SIGN_DIST && dist > 0) {
                    std::cout << "traffic light detected at a distance of: " << dist << std::endl;
                    detected_dist = dist;
                    stopsign_flag = true;
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
};

void StateMachine::run() {
    while (ros::ok()) {
        if (mpc.target_waypoint_index >= mpc.num_waypoints -1) change_state(STATE::DONE);
        if (state == STATE::MOVING) {
            // check if stopline is reached
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
                        if (ekf) {
                            utils.get_ekf_states(x, y, yaw);
                        } else {
                            utils.get_gps_states(x, y, yaw);
                        }
                        auto car_pose = utils.estimate_object_pose2d(x, y, yaw, bbox, dist, utils.CAMERA_PARAMS);
                        printf("estimated car pose: %3f, %3f\n", car_pose[0], car_pose[1]);
                        // compute distance from detected car to closest waypoint in front of car to assess whether car is in same lane
                        double look_ahead_dist = dist * 1.5;
                        int look_ahead_index = look_ahead_dist * mpc.density + mpc.closest_waypoint_index;
                        // compute distance from car_pose to waypoint, find closest waypoint and distance
                        double min_dist_sq = 1000.;
                        int min_index = 0;
                        for (int i = mpc.closest_waypoint_index; i < look_ahead_index; i++) {
                            double dist_sq = (car_pose.head(2) - mpc.state_refs.row(i).head(2)).squaredNorm();
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
            double offset = std::max(0.0, detected_dist - MIN_SIGN_DIST);
            ROS_INFO("stop sign detected at a distance of: %3f, offset: %3f", detected_dist, offset);
            double x0, y0, yaw0;
            if (ekf) {
                utils.get_ekf_states(x0, y0, yaw0);
            } else {
                utils.get_gps_states(x0, y0, yaw0);
            }
            while(1) {
                double x, y, yaw;
                if (ekf) {
                    utils.get_ekf_states(x, y, yaw);
                } else {
                    utils.get_gps_states(x, y, yaw);
                }
                double norm = std::sqrt((x0 - x)*(x0 - x) + (y0 - y)*(y0 - y));
                ROS_INFO("norm: %3f", norm);
                if (norm >= offset)
                {
                    ROS_INFO("waiting for stop sign...");
                    utils.publish_cmd_vel(0.0, 0.0);
                    break;
                }
                update_mpc_state();
                solve();
                rate->sleep();
            }
            change_state(STATE::WAITING_FOR_STOPSIGN);
            continue;
        } else if (state == STATE::LANE_FOLLOWING) {
            update_mpc_state();
            double error_sq = (mpc.x_current.head(2) - destination).squaredNorm();
            if (error_sq < TOLERANCE_SQUARED) {
                change_state(STATE::DONE);
            }
            double steer = utils.get_steering_angle();
            double speed = 0.5;
            utils.publish_cmd_vel(steer, speed);
            rate->sleep();
            continue;
        } else if (state == STATE::WAITING_FOR_STOPSIGN) {
            stop_for(STOP_DURATION);
            cooldown_timer = ros::Time::now() + ros::Duration(SIGN_COOLDOWN);
            change_state(STATE::MOVING);
        } else if (state == STATE::WAITING_FOR_LIGHT) {
            stop_for(STOP_DURATION);
            cooldown_timer = ros::Time::now() + ros::Duration(SIGN_COOLDOWN);
            change_state(STATE::MOVING);
        } else if (state == STATE::PARKING) {
            // double offset = detected_dist + PARK_OFFSET;
            // if(sign) {
            //     int car_index = utils.object_index(OBJECT::CAR);
            //     if (car_index >= 0) {
            //         double car_dist = utils.object_distance(car_index);
            //         double dist_to_first_spot = detected_dist + PARKSIGN_TO_CAR - CAR_LENGTH/2;
            //         if (car_dist - dist_to_first_spot > (PARKING_SPOT_LENGTH + CAR_LENGTH)/2 * 1.1) { // 1.1 is a safety factor
            //             ROS_INFO("car parked in second spot, proceed to park in first spot");
            //         } else {
            //             ROS_INFO("car parked in first spot, proceed to park in second spot");
            //             offset += PARKING_SPOT_LENGTH;
            //         }
            //     }
            // } else offset = 0.0;
            double offset_thresh = 0.1;
            double base_offset = detected_dist + PARKING_SPOT_LENGTH * 1.5 + offset_thresh;
            // base_offset = 0;
            right_park = true;
            ROS_INFO("parking offset is: %3f", base_offset);
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
            stop_for(STOP_DURATION/2);
            bool hard_code = true;
            if (hard_code) {
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
                parking_maneuver_hardcode(right_park, true, 1/T_park);
            } else {
                xs << -0.2, 0., 0. , 0., 0.;
                ROS_INFO("moving back for %3f", xs[0]);
                move_to(xs, 0.05);
                xs << 0.63, 0.4, 0., 0., 0.;
                move_to(xs, 0.15);
            }
            change_state(STATE::MOVING);
        } else if (state == STATE::INIT) {
            change_state(STATE::MOVING);
            if (lane) change_state(STATE::LANE_FOLLOWING);
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
int StateMachine::parking_maneuver_hardcode(bool right, bool exit, double rate_val) {
    double x0, y0, yaw0;
    // get current states
    utils.get_states(x0, y0, yaw0);
    // get closest direction
    yaw0 = mpc.NearestDirection(yaw0);
    while (yaw0 < -M_PI) yaw0 += 2*M_PI;
    while (yaw0 > M_PI) yaw0 -= 2*M_PI;
    int stage = 1;
    double steering_angle = -23.;
    double speed = -0.3;
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
        if (stage == 1 && std::abs(yaw_error) > M_PI/4) {
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
        // printf("yaw: %3f, yaw_error: %3f, steering_angle: %3f\n", yaw, yaw_error, steering_angle);
        utils.publish_cmd_vel(steering_angle, speed);
        temp_rate.sleep();
    }
    return 0;
}
void StateMachine::update_mpc_state() {
    double x, y, yaw;
    if (ekf) {
        utils.get_ekf_states(x, y, yaw);
        mpc.update_current_states(utils.gps_x, utils.gps_y, utils.yaw, mpc.x_real); // update real states as well
    } else {
        utils.get_gps_states(x, y, yaw);
    }
    mpc.update_current_states(x, y, yaw); // no argument means use current states
    if(debug) {
        ;
    }
}
void StateMachine::solve() {
    // auto start = std::chrono::high_resolution_clock::now();

    int status = mpc.update_and_solve();
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

StateMachine *globalStateMachinePtr = nullptr;
void signalHandler(int signum) {
    if (globalStateMachinePtr) {
        globalStateMachinePtr->utils.stop_car();
        globalStateMachinePtr->mpc.computeStats(357);
        globalStateMachinePtr->utils.print_detected_cars();
    }
    ros::shutdown();
    exit(signum);
}

using json = nlohmann::json;
int main(int argc, char **argv) {
    std::string dir = Optimizer::getSourceDirectory();
    //replace last occurence of src with scripts
    dir.replace(dir.rfind("src"), 3, "scripts");
    std::string file_path = dir + "/" + "config/mpc_config2.json";
    std::ifstream file(file_path);
    if(!file.is_open()) {
        std::cout << "Failed to open file: " << file_path << std::endl;
        exit(1);
    }
    json j;
    file >> j;

    std::cout.precision(3);
    ros::init(argc, argv, "mpc_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    double T, v_ref, T_park;
    int N;
    bool sign, ekf, lane;
    std::string name;
    bool success = nh.getParam("/mpc_controller/lane", lane) && nh.getParam("/mpc_controller/ekf", ekf) && nh.getParam("/mpc_controller/sign", sign) && nh.getParam("T", T) && nh.getParam("N", N) && nh.getParam("constraints/v_ref", v_ref);
    success = success && nh.getParam("/mpc_controller/name", name);
    success = success && nh.getParam("/T_park", T_park);
    if (!success) {
        std::cout << "Failed to get parameters" << std::endl;
        T = 0.100;
        N = 40;
        v_ref = 1.0;
        sign = false;
        ekf = false;
        lane = false;
        exit(1);
    } else {
        std::cout << "Successfully got parameters" << std::endl;
    }
    std::cout << "ekf: " << ekf << ", sign: " << sign << ", T: " << T << ", N: " << N << ", v_ref: " << v_ref << std::endl;
    StateMachine sm(nh, T, N, v_ref, sign, ekf, lane, T_park, name);

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