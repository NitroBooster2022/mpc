#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>
#include <array>

namespace VehicleConstants {
    const std::array<std::string, 3> MANEUVER_DIRECTIONS = {"left", "straight", "right"};

    const std::array<std::string, 12> state_names = {
        "INIT", "MOVING", "APPROACHING_INTERSECTION", "WAITING_FOR_STOPSIGN",
        "WAITING_FOR_LIGHT", "PARKING", "PARKED", "EXITING_PARKING", "DONE", "LANE_FOLLOWING", "INTERSECTION_MANEUVERING", "KEYBOARD"
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
        INTERSECTION_MANEUVERING,
        KEYBOARD_CONTROL
    };
    enum STOPSIGN_FLAGS {
        NONE,
        STOP,
        LIGHT,
        PRIO,
        RDB
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
    enum MANEUVER_DIRECTION {
        LEFT,
        STRAIGHT,
        RIGHT
    };

    static constexpr double CAM_TO_CAR_FRONT = 0.21;
    static constexpr double CAR_LENGTH = 0.464;
    static constexpr double CAR_WIDTH = 0.1885;
    static constexpr double CAR_HEIGHT = 0.1155;
    static constexpr double MAX_TAILING_DIST = 1.0;
    static constexpr double MIN_SIGN_DIST = 0.39;  // 0.6 - 0.21
    // static constexpr double MAX_SIGN_DIST = 1.09;
    static constexpr double MAX_SIGN_DIST = 0.753;
    // static constexpr double MAX_PARK_DIST = 0.79;  // 1.0 - 0.21
    static constexpr double MAX_PARK_DIST = 1.;  // 1.0 - 0.21
    static constexpr double MAX_CROSSWALK_DIST = 0.79;  // 1.0 - 0.21
    static constexpr double PARKSIGN_TO_CAR = 0.51;
    static constexpr double PARK_OFFSET = 1.31;    // 1.1 + 0.21
    static constexpr double PARKING_SPOT_LENGTH = 0.723;
    static constexpr double PARKING_SPOT_WIDTH = 0.362;
    static constexpr double MAX_PARKING_Y_ERROR = 0.0775;
    static constexpr double CROSSWALK_LENGTH = 1.0;
    static constexpr double OVERTAKE_DIST = 2.0;
    static constexpr double LANE_OFFSET = 0.36;
    static constexpr double MIN_DIST_TO_CAR = 0.3;
    static constexpr double MAX_CAR_DIST = 3.0;
    static constexpr double SIGN_COOLDOWN = 1.0;
    static constexpr double TOLERANCE_SQUARED = 0.01;
    static constexpr double STOP_DURATION = 1.50;
    static constexpr double NORMAL_SPEED = 0.175;
    static constexpr double FAST_SPEED = 0.4;
    static constexpr double SOFT_MAX_STEERING = 0.42 * 180 / M_PI;
    static constexpr double HARD_MAX_STEERING = 25.0;
    static constexpr std::array<double, 2> PARKING_SPOT_RIGHT = {9.50, 0.372};
    static constexpr std::array<double, 2> PARKING_SPOT_LEFT = {9.50, 1.110};
    static constexpr std::array<double, 2> PARKING_SIGN_TO_CAR1 = {0.4592, -0.073};
    static constexpr std::array<std::array<double, 2>, 2> PARKING_SIGN_POSES = {{{{8.922497, 0.487}}, {{8.922405, 1.008329}}}};
    static constexpr std::array<std::array<double, 2>, 4> ROUNDABOUT_POSES = {{{{14.906, 10.190544}}, {{16.5132, 9.455325}}, {{17.247, 11.067}}, {{15.639, 11.80325}}}};
    static constexpr std::array<std::array<double, 2>, 5> CROSSWALK_POSES = {{{{17.365101, 2.282282}}, {{8.125406, 0.490722}}, {{8.914196, 3.406469}}, {{9.582251, 4.291623}}, {{1.833610, 10.3011}}}};
    static constexpr std::array<std::array<double, 2>, 11> INTERSECTION_SIGN_POSES = {{{{0.067568, 4.669999}}, {{1.90, 0.444024}}, {{3.207346, 3.032455}}, {{3.206, 5.9274}}, {{4.146563, 6.259}}, {{0.072, 10.682}}, {{15.016, 4.67363}}, {{7.6468, 4.33}}, {{5.791, 4.33}}, {{4.4838, 1.752361}}, {{6.06227, 0.511846}}}};

    //utils
    static constexpr int NUM_VALUES_PER_OBJECT = 7;
    enum SignValues { x1, y1, x2, y2, distance, confidence, id };
    enum LOCALIZATION_SOURCE {
        ODOM,
        EKF
    };
    static constexpr std::array<double, 4> CAMERA_PARAMS = {554.3826904296875, 554.3826904296875, 320, 240};
    //   <pose>0 0.05 0.2 0 0.2617 -0.</pose>
    static constexpr std::array<double, 6> REALSENSE_TF = {0, 0.05, 0.2, 0, 0.2617, 0};
}

#endif // VEHICLE_CONSTANTS_H
