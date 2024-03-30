#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>
#include <array>

namespace VehicleConstants {
    const std::array<std::string, 3> MANEUVER_DIRECTIONS = {"left", "straight", "right"};

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
    static constexpr double MIN_DIST_TO_CAR = 0.8;
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
