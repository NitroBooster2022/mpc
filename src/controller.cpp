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
    StateMachine(ros::NodeHandle& nh_) : nh(nh_), utils(nh), cooldown_timer(ros::Time::now())
    {
        double rateVal = 1/mpc.T;
        rate = new ros::Rate(rateVal);
        std::cout << "rate: " << rateVal << std::endl;
        x0 = {0, 0, 0};
        state = STATE::INIT;
    }
    ~StateMachine() {
        // utils.stop_car();
    }
    ros::NodeHandle& nh;

// private:
    // Constants
    enum STATE {
        INIT,
        MOVING,
        APPRAOCHING_INTERSECTION,
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
    static constexpr double MIN_SIGN_DIST = 0.39;  // 0.6 - 0.21
    static constexpr double MAX_SIGN_DIST = 1.09;  // 1.3 - 0.21
    static constexpr double MAX_PARK_DIST = 0.79;  // 1.0 - 0.21
    static constexpr double PARKSIGN_TO_CAR = 0.51;
    static constexpr double PARK_OFFSET = 1.31;    // 1.1 + 0.21
    static constexpr double PARKING_SPOT_LENGTH = 0.723;
    static constexpr double OVERTAKE_DIST = 2.0;
    static constexpr double LANE_OFFSET = 0.31;
    static constexpr double MIN_DIST_TO_CAR = 0.8;
    static constexpr double MAX_CAR_DIST = 1.8;
    static constexpr double SIGN_COOLDOWN = 1.0;
    
    double detected_dist = 0;
    int state = 0;
    bool debug = true;
    std::string gaz_bool = "_gazebo_";

    ros::Time cooldown_timer;
    std::array<double, 3> x0;
    ros::Rate* rate;

    std::mutex lock;
    Utility utils;
    Optimizer mpc;

    void solve();
    void update_mpc_state();
    void change_state(STATE new_state);
    void run();
};

void StateMachine::run() {
    while (ros::ok()) {
        if (mpc.target_waypoint_index >= mpc.num_waypoints -1) change_state(STATE::DONE);
        if (state == STATE::MOVING) {
            update_mpc_state();
            solve();
            rate->sleep();
        } else if (state == STATE::INIT) {
            change_state(STATE::MOVING);
        } else if (state == STATE::DONE) {
            std::cout << "Done" << std::endl;
            break;
        }
    }
}
void StateMachine::update_mpc_state() {
    mpc.update_current_states(utils.gps_x, utils.gps_y, utils.yaw);
    if(debug) {
        ;
    }
}
void StateMachine::solve() {
    int status = mpc.update_and_solve();
    double steer = -mpc.u_current[1] * 180 / M_PI; // convert to degrees
    double speed = mpc.u_current[0];
    // steer = 23;
    // speed = 0.573;
    utils.publish_cmd_vel(steer, speed);
    if (debug) {
        ;
    }
}
void StateMachine::change_state(STATE new_state) {
    std::cout << "Changing from " << state << " to " << new_state << std::endl;
    state = new_state;
}

StateMachine *globalStateMachinePtr = nullptr;
void signalHandler(int signum) {
    if (globalStateMachinePtr) {
        globalStateMachinePtr->utils.stop_car();
    }

    ros::shutdown();
    exit(signum);
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "mpc_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    std::cout.precision(3);
    StateMachine sm(nh);

    globalStateMachinePtr = &sm;
    signal(SIGINT, signalHandler);

    std::thread t(&StateMachine::run, &sm);
    std::thread t2(&Utility::spin, &sm.utils);
    t.join();
    t2.join();
    std::cout << "threads joined" << std::endl;
    return 0;
}