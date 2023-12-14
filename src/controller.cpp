#include <ros/ros.h>
#include <thread>
#include <map>
#include <string>
#include <vector>
#include <mutex>
#include "utility.hpp"
#include "optimizer.hpp"
#include <signal.h>

class StateMachine {
public:
    StateMachine(ros::NodeHandle& nh_, double T, int N, double v_ref, bool sign, bool ekf, double T_park): 
    nh(nh_), utils(nh, sign, ekf), mpc(T,N,v_ref), cooldown_timer(ros::Time::now()), xs(5),
    state(STATE::INIT), sign(sign), ekf(ekf), T_park(T_park), T(T)
    {
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
    const std::array<std::string, 9> state_names = {
        "INIT", "MOVING", "APPROACHING_INTERSECTION", "WAITING_FOR_STOPSIGN",
        "WAITING_FOR_LIGHT", "PARKING", "PARKED", "EXITING_PARKING", "DONE"
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
    struct CameraParams {
        double fx;
        double fy;
        int cx;
        int cy;
    } const CAMERA_PARAMS = {554.3826904296875, 554.3826904296875, 320, 240};
    struct CameraPose {
        double x;
        double y;
        double z;
    } const CAMERA_POSE = {0, 0, 0.2};
    static constexpr double CAM_TO_CAR_FRONT = 0.21;
    static constexpr double CAR_LENGTH = 0.464;
    static constexpr double MAX_TAILING_DIST = 0.75;
    static constexpr double MIN_SIGN_DIST = 0.39;  // 0.6 - 0.21
    static constexpr double MAX_SIGN_DIST = 1.09;  // 1.3 - 0.21
    static constexpr double MAX_PARK_DIST = 0.79;  // 1.0 - 0.21
    static constexpr double PARKSIGN_TO_CAR = 0.51;
    static constexpr double PARK_OFFSET = 1.31;    // 1.1 + 0.21
    static constexpr double PARKING_SPOT_LENGTH = 0.723;
    static constexpr double OVERTAKE_DIST = 2.0;
    static constexpr double LANE_OFFSET = 0.36;
    static constexpr double MIN_DIST_TO_CAR = 0.8;
    static constexpr double MAX_CAR_DIST = 1.8;
    static constexpr double SIGN_COOLDOWN = 1.0;
    static constexpr double TOLERANCE_SQUARED = 0.01;
    
    double T_park, T;
    double detected_dist = 0;
    int detected_index = 0;
    Eigen::Vector2d destination;
    Eigen::VectorXd xs;
    int state = 0;
    bool debug = true, sign, ekf;
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
    int check_car();
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
};

void StateMachine::run() {
    while (ros::ok()) {
        if (mpc.target_waypoint_index >= mpc.num_waypoints -1) change_state(STATE::DONE);
        if (state == STATE::MOVING) {
            if (sign) {
                double dist;
                int stopsign_index = utils.object_index(OBJECT::STOPSIGN);
                if(stopsign_index >= 0) {
                    dist = utils.object_distance(stopsign_index);
                    if (ros::Time::now() > cooldown_timer && dist > 0) {
                        std::cout << "stop sign detected at a distance of: " << dist << std::endl;
                        // auto box = utils.object_box(stopsign_index);
                        // auto world_pose = utils.estimate_object_pose2d(utils.get_real_states(), box, dist, CAMERA_PARAMS);
                        // std::cout << "world pose: " << world_pose << std::endl;
                        // utils.add_object("stopsign", world_pose);
                        detected_dist = dist;
                        change_state(STATE::APPROACHING_INTERSECTION);
                        continue;
                    }
                }
                int light_index = utils.object_index(OBJECT::LIGHTS);
                if(light_index >= 0) {
                    dist = utils.object_distance(light_index);
                    if (ros::Time::now() > cooldown_timer && dist > 0) {
                        std::cout << "traffic light detected at a distance of: " << dist << std::endl;
                        // auto box = utils.object_box(stopsign_index);
                        // auto world_pose = utils.estimate_object_pose2d(utils.get_real_states(), box, dist, CAMERA_PARAMS);
                        // std::cout << "world pose: " << world_pose << std::endl;
                        // utils.add_object("light", world_pose);
                        detected_dist = dist;
                        change_state(STATE::APPROACHING_INTERSECTION);
                        continue;
                    }
                }
                int park_index = utils.object_index(OBJECT::PARK);
                if(park_index >= 0) {
                    dist = utils.object_distance(park_index);
                    if (dist < MAX_PARK_DIST && dist > 0) {
                        std::cout << "parking sign detected at a distance of: " << dist << std::endl;
                        change_state(STATE::PARKING);
                        continue;
                    }
                }
                // check_car();
                int car_index = utils.object_index(OBJECT::CAR);
                if(car_index >= 0) {
                    dist = utils.object_distance(car_index);
                    std::cout << "car dist: " << dist << std::endl;
                    auto attribute = mpc.get_current_attribute();
                    if (attribute == mpc.ATTRIBUTE::ONEWAY || attribute == mpc.ATTRIBUTE::CROSSWALK) {
                        if (dist < MAX_TAILING_DIST && dist > 0) {
                            std::cout << "car detected on oneway, stopping" << std::endl;
                            stop_for(T);
                        }
                    } else if (dist < MAX_CAR_DIST && mpc.target_waypoint_index >= detected_index * 1.2 && park_index < 0 && dist > 0) {
                        dist = std::max(dist + CAM_TO_CAR_FRONT, MIN_DIST_TO_CAR) - MIN_DIST_TO_CAR;
                        std::cout << "car detected at a distance of: " << dist << ", changing lane." << std::endl;
                        int offset = static_cast<int>(dist * mpc.density);
                        detected_index = mpc.target_waypoint_index + static_cast<int>(OVERTAKE_DIST * mpc.density);
                        mpc.change_lane(mpc.target_waypoint_index+offset, detected_index+offset, false, LANE_OFFSET);
                        continue;
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
        } else if (state == STATE::WAITING_FOR_STOPSIGN) {
            stop_for(3.0);
            cooldown_timer = ros::Time::now() + ros::Duration(SIGN_COOLDOWN);
            change_state(STATE::MOVING);
        } else if (state == STATE::WAITING_FOR_LIGHT) {
            stop_for(3.0);
            cooldown_timer = ros::Time::now() + ros::Duration(SIGN_COOLDOWN);
            change_state(STATE::MOVING);
        } else if (state == STATE::PARKING) {
            double offset = detected_dist + PARK_OFFSET;
            if(sign) {
                int car_index = utils.object_index(OBJECT::CAR);
                if (car_index >= 0) {
                    double car_dist = utils.object_distance(car_index);
                    double dist_to_first_spot = detected_dist + PARKSIGN_TO_CAR - CAR_LENGTH/2;
                    if (car_dist - dist_to_first_spot > (PARKING_SPOT_LENGTH + CAR_LENGTH)/2 * 1.1) { // 1.1 is a safety factor
                        ROS_INFO("car parked in second spot, proceed to park in first spot");
                    } else {
                        ROS_INFO("car parked in first spot, proceed to park in second spot");
                        offset += PARKING_SPOT_LENGTH;
                    }
                }
            } else offset = 0.0;
            ROS_INFO("parking offset is: %3f", offset);
            xs << offset, 0., 0., 0., 0.;
            ROS_INFO("moving to: %3f, %3f, %3f", xs[0], xs[1], xs[2]);
            move_to(xs);
            xs << -0.63, -0.35, 0. , 0., 0.;
            ROS_INFO("moving to: %3f, %3f, %3f", xs[0], xs[1], xs[2]);
            move_to(xs);
            change_state(STATE::PARKED);
        } else if (state == STATE::PARKED) {
            stop_for(3.0);
            change_state(STATE::EXITING_PARKING);
        } else if (state == STATE::EXITING_PARKING) {
            xs << -0.2, 0., 0. , 0., 0.;
            ROS_INFO("moving back for %3f", xs[0]);
            move_to(xs, 0.05);
            xs << 0.63, 0.4, 0., 0., 0.;
            move_to(xs, 0.15);
            change_state(STATE::MOVING);
        } else if (state == STATE::INIT) {
            change_state(STATE::MOVING);
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
int StateMachine::check_car() {
    // int car_index = utils.object_index(OBJECT::CAR);
    // if(car_index >= 0) {
    //     int dist = utils.object_distance(car_index);
    //     std::cout << "car dist: " << dist << std::endl;
    //     if (dist < MAX_CAR_DIST && mpc.target_waypoint_index >= detected_index * 1.2 && park_index < 0 && dist > 0) {
    //         dist = std::max(dist + CAM_TO_CAR_FRONT, MIN_DIST_TO_CAR) - MIN_DIST_TO_CAR;
    //         std::cout << "car detected at a distance of: " << dist << std::endl;
    //         int offset = static_cast<int>(dist * mpc.density);
    //         detected_index = mpc.target_waypoint_index + static_cast<int>(OVERTAKE_DIST * mpc.density);
    //         mpc.change_lane(mpc.target_waypoint_index+offset, detected_index+offset, false, LANE_OFFSET);
    //         return 0;
    //     }
    // }
}

StateMachine *globalStateMachinePtr = nullptr;
void signalHandler(int signum) {
    if (globalStateMachinePtr) {
        globalStateMachinePtr->utils.stop_car();
        globalStateMachinePtr->mpc.computeStats(357);
    }
    ros::shutdown();
    exit(signum);
}

int main(int argc, char **argv) {
    std::cout.precision(3);
    ros::init(argc, argv, "mpc_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    double T, v_ref, T_park;
    int N;
    bool sign, ekf;
    bool success = nh.getParam("/mpc_controller/ekf", ekf) && nh.getParam("/mpc_controller/sign", sign) && nh.getParam("T", T) && nh.getParam("N", N) && nh.getParam("constraints/v_ref", v_ref);
    success = success && nh.getParam("/T_park", T_park);
    if (!success) {
        std::cout << "Failed to get parameters" << std::endl;
        T = 0.100;
        N = 40;
        v_ref = 1.0;
        sign = false;
        ekf = false;
        exit(1);
    } else {
        std::cout << "Successfully got parameters" << std::endl;
    }
    std::cout << "ekf: " << ekf << ", sign: " << sign << ", T: " << T << ", N: " << N << ", v_ref: " << v_ref << std::endl;
    StateMachine sm(nh, T, N, v_ref, sign, ekf, T_park);

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