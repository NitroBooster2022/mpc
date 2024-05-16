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
    enum DETECTED_CAR_STATE {
        SAME_LANE,
        ADJACENT_LANE,
        NOT_SURE
    };

    static constexpr double CAM_TO_CAR_FRONT = 0.21;
    static constexpr double CAR_LENGTH = 0.464;
    static constexpr double CAR_WIDTH = 0.1885;
    static constexpr double CAR_HEIGHT = 0.1155;
    static constexpr double LANE_CENTER_TO_EDGE = 0.0777;
    static constexpr int IMAGE_HEIGHT = 480;
    static constexpr int IMAGE_WIDTH = 640;
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
    static constexpr double LANE_OFFSET = 0.3935;
    static constexpr double INNER_LANE_OFFSET = 0.3465;
    static constexpr double MIN_DIST_TO_CAR = 0.3;
    static constexpr double MAX_CAR_DIST = 3.0;
    static constexpr double SIGN_COOLDOWN = 1.0;
    static constexpr double TOLERANCE_SQUARED = 0.01;
    // static constexpr double STOP_DURATION = 1.50;
    // static constexpr double NORMAL_SPEED = 0.175;
    // static constexpr double FAST_SPEED = 0.4;
    static constexpr double SOFT_MAX_STEERING = 0.42 * 180 / M_PI;
    static constexpr double HARD_MAX_STEERING = 25.0;

    static constexpr double pole_size = 0.0514;

    // parking coordinates
    static constexpr std::array<double, 2> PARKING_SPOT_RIGHT = {9.07365, 0.703 - INNER_LANE_OFFSET/2 + pole_size/2};
    static constexpr std::array<double, 2> PARKING_SPOT_LEFT = {9.07365, 1.1527 + INNER_LANE_OFFSET/2 - pole_size/2};

    static constexpr double ofs6 = INNER_LANE_OFFSET/2 - pole_size/2;
    static constexpr double hsw = pole_size/2;
    static constexpr double haha = 369. - INNER_LANE_OFFSET;
    // add half of inner lane width to the x values
    // static constexpr std::array<double, 13> Y_ALIGNED_LANE_CENTERS = {0.22237, 0.591617, 2.383851, 2.754291, 4.63, 4.9981, 6.49, 6.864, 15.17, 16.963, 15.54, 15.7404, 16.112};
    static const std::vector<double> NORTH_FACING_LANE_CENTERS = {0.579612+ofs6, 2.744851+ofs6, 4.9887+ofs6, 6.77784+ofs6, 6.8507+ofs6, 16.954+ofs6, 15.532+ofs6, 16.1035+ofs6};
    static const std::vector<double> SOUTH_FACING_LANE_CENTERS = {0.50684-ofs6, 2.667-ofs6, 4.9156-ofs6, 15.165279+ofs6, 15.1632+ofs6};
    // add half of inner lane width to the y values
    // static constexpr std::array<double, 13> X_ALIGNED_LANE_CENTERS = {13.314624, 12.94356, 10.669, 10.2963, 3.89, 0.598716, 0.9698, 3.516515, 3.88667, 6.4122, 6.78514, 11.6955, 12.0661};
    static const std::vector<double> EAST_FACING_LANE_CENTERS = {12.904+ofs6, 10.5538-ofs6, 0.503891-ofs6, 3.79216-ofs6, 6.6816-ofs6, 10.5538-ofs6};
    static const std::vector<double> WEST_FACING_LANE_CENTERS = {13.314624+ofs6, 10.633+ofs6, 3.86375+ofs6, 0.58153+ofs6, 3.8661+ofs6, 6.753+ofs6, 13.278+ofs6};

    
    // intersection coordinates
    static const std::vector<std::vector<double>> SOUTH_FACING_INTERSECTIONS = {
        {{16.075438-ofs6, 11.684077-hsw}},
        {{15.46-ofs6, 4.58-hsw}},
        {{15.165376+ofs6, 1.4972-hsw}},
        {{4.9156-ofs6, 4.581664-hsw}},
        {{4.9143-ofs6, 1.3-hsw}},
        {{2.667-ofs6, 1.3-hsw}},
        {{2.67-ofs6, 4.584-hsw}},
        {{0.50684-ofs6, 4.5849-hsw}},
        {{0.50684-ofs6, 7.470675-hsw}},
        {{0.50684-ofs6, 10.584-hsw}}
    };
    static const std::vector<std::vector<double>> NORTH_FACING_INTERSECTIONS = {
        {{0.579612+ofs6, 9.0727+hsw}},
        {{0.579612+ofs6, 5.96247+hsw}},
        {{0.579612+ofs6, 3.07145+hsw}},
        {{2.744851+ofs6, 3.07145+hsw}},
        {{2.744851+ofs6, 5.9603+hsw}},
        {{4.9887+ofs6, 5.958+hsw}},
        {{4.9887+ofs6, 3.07+hsw}},
        {{6.77784-ofs6, 3.44261+hsw}},
        {{16.104+ofs6, 9.5053+hsw}}
    };
    static const std::vector<std::vector<double>> WEST_FACING_INTERSECTIONS = {
        {{17.1571-hsw, 10.633+ofs6}},
        {{1.296-hsw, 3.86375+ofs6}},
        {{3.4543-hsw, 3.865637+ofs6}},
        {{5.71-hsw, 3.8661+ofs6}},
        {{5.708-hsw, 6.753+ofs6}},
        {{3.4547-hsw, 6.7545+ofs6}},
        {{1.296-hsw, 6.754754+ofs6}},
        {{7.568552-hsw, 3.8674+ofs6}},
        {{3.45624-hsw, 0.58153+ofs6}},
        {{16.2485-hsw, 3.8678+ofs6}}
    };
    static const std::vector<std::vector<double>> EAST_FACING_INTERSECTIONS = {
        {{1.95075+hsw, 0.503891-ofs6}},
        {{1.95075+hsw, 3.794-ofs6}}, 
        {{1.95+hsw, 6.6816-ofs6}}, 
        {{4.19476+hsw, 6.681-ofs6}}, 
        {{4.19476+hsw, 3.79216-ofs6}}, 
        {{4.194644+hsw, 0.503836-ofs6}}, 
        // {{6.0735+hsw, 0.5949+ofs6}}, 
        {{14.7386+hsw, 1.07135-ofs6}}, 
        {{14.983+hsw, 10.5538-ofs6}}
    };

    static constexpr double ofs7 = 0.445;
    static constexpr double ofs3 = 0.062;
    // 5.1y 5.6x
    static constexpr double sign_ofs1 = 0.056;
    static constexpr double sign_ofs2 = 0.051;
    static const std::vector<std::vector<double>> SOUTH_FACING_SIGNS = {
        {{15.46-ofs6*2-pole_size-sign_ofs1, 4.58+sign_ofs2}},
        {{15.165376-pole_size-sign_ofs1, 1.4972+sign_ofs2}},
        {{4.9156-ofs6*2-pole_size-sign_ofs1, 4.581664+sign_ofs2}},
        {{4.9143-ofs6*2-pole_size-sign_ofs1, 1.3+sign_ofs2}},
        {{2.667-ofs6*2-pole_size-sign_ofs1, 1.3+sign_ofs2}},
        {{2.67-ofs6*2-pole_size-sign_ofs1, 4.584+sign_ofs2}},
        {{0.50684-ofs6*2-pole_size-sign_ofs1, 4.5849+sign_ofs2}},
        {{0.50684-ofs6*2-pole_size-sign_ofs1, 7.470675+sign_ofs2}},
        {{0.50684-ofs6*2-pole_size-sign_ofs1, 10.584+sign_ofs2}}
    };
    static const std::vector<std::vector<double>> NORTH_FACING_SIGNS = {
        {{0.579612+ofs6*2+pole_size+sign_ofs1, 9.0727-sign_ofs2}},
        {{0.579612+ofs6*2+pole_size+sign_ofs1, 5.96247-sign_ofs2}},
        {{0.579612+ofs6*2+pole_size+sign_ofs1, 3.07145-sign_ofs2}},
        {{2.744851+ofs6*2+pole_size+sign_ofs1, 3.07145-sign_ofs2}},
        {{2.744851+ofs6*2+pole_size+sign_ofs1, 5.9603-sign_ofs2}},
        {{4.9887+ofs6*2+pole_size+sign_ofs1, 5.958-sign_ofs2}},
        {{4.9887+ofs6*2+pole_size+sign_ofs1, 3.07-sign_ofs2}},
        {{6.77784-ofs6*2+pole_size+sign_ofs1, 3.44261-sign_ofs2}},
    };

    static const std::vector<std::vector<double>> WEST_FACING_SIGNS = {
        {{17.1571+sign_ofs2, 10.633+ofs6*2+pole_size+sign_ofs1}},
        {{1.296+sign_ofs2, 3.86375+ofs6*2+pole_size+sign_ofs1}},
        {{3.4543+sign_ofs2, 3.865637+ofs6*2+pole_size+sign_ofs1}},
        {{5.71+sign_ofs2, 3.8661+ofs6*2+pole_size+sign_ofs1}},
        {{5.708+sign_ofs2, 6.753+ofs6*2+pole_size+sign_ofs1}},
        {{3.4547+sign_ofs2, 6.7545+ofs6*2+pole_size+sign_ofs1}},
        {{1.296+sign_ofs2, 6.754754+ofs6*2+pole_size+sign_ofs1}},
        {{7.568552+sign_ofs2, 3.8674+ofs6*2+pole_size+sign_ofs1}},
        {{3.45624+sign_ofs2, 0.58153+ofs6*2+pole_size+sign_ofs1}},
        {{16.2485+sign_ofs2, 3.8678+ofs6*2+pole_size+sign_ofs1}}
    };

    static const std::vector<std::vector<double>> EAST_FACING_SIGNS = {
        {{1.95075-sign_ofs2, 0.503891-ofs6*2-pole_size-sign_ofs1}},
        {{1.95075-sign_ofs2, 3.794-ofs6*2-pole_size-sign_ofs1}}, 
        {{1.95-sign_ofs2, 6.6816-ofs6*2-pole_size-sign_ofs1}}, 
        {{4.19476-sign_ofs2, 6.681-ofs6*2-pole_size-sign_ofs1}}, 
        {{4.19476-sign_ofs2, 3.79216-ofs6*2-pole_size-sign_ofs1}}, 
        {{4.194644-sign_ofs2, 0.503836-ofs6*2-pole_size-sign_ofs1}}, 
        {{14.7386-sign_ofs2, 1.07135-ofs6*2-pole_size-sign_ofs1}}, 
        {{14.983-sign_ofs2, 10.5538-ofs6*2-pole_size-sign_ofs1}}
    };

    // PARKING SIGN COORDINATES
    static constexpr double park_ofs1_left = 0.009, park_ofs1_right = 0.016;
    static constexpr double park_ofs2 = 0.05325;
    static const std::vector<std::vector<double>> PARKING_SIGN_POSES = {{{{8.99-park_ofs2, 0.703367-park_ofs1_right}}, {{8.99-park_ofs2, 1.1522+park_ofs1_left}}}};
    
    // ROUNDABOUT COORDINATES
    static constexpr double rdb_ofs1 = 0.107834;
    static constexpr double rdb_ofs2 = 0.05361;
    static const std::vector<std::vector<double>> ROUNDABOUT_POSES = {{{{14.9777, 10.263}}, {{16.3974, 9.455325}}, {{17.247, 11.067}}, {{15.639, 11.80325}}}};
    static const std::vector<std::vector<double>> EAST_FACING_ROUNDABOUT = {{{14.9777-rdb_ofs2, 10.263-rdb_ofs1}}};
    static const std::vector<std::vector<double>> NORTH_FACING_ROUNDABOUT = {{{16.4+rdb_ofs1, 9.52-rdb_ofs2}}};
    static const std::vector<std::vector<double>> WEST_FACING_ROUNDABOUT = {{{17.164+rdb_ofs2, 10.928+rdb_ofs1}}};
    static const std::vector<std::vector<double>> SOUTH_FACING_ROUNDABOUT = {{{15.737-rdb_ofs1, 11.690741+rdb_ofs2}}};

    // CROSSWALK COORDINATES
    static constexpr double ofs2 = 0.05361;
    static constexpr double ofs1 = 0.107834;
    // static const std::vector<std::vector<double>> CROSSWALK_POSES = {{{{17.365101, 2.282282}}, {{8.125406, 0.490722}}, {{8.914196, 3.406469}}, {{9.582251, 4.291623}}, {{1.833610, 10.3011}}}};
    static const std::vector<std::vector<double>> EAST_FACING_CROSSWALKS = {{{{8.178-ofs2, 0.600-ofs1}},{{1.2116-ofs2, 5.515537-ofs1}}}};
    static const std::vector<std::vector<double>> WEST_FACING_CROSSWALKS = {{{{1.77+ofs2, 10.196+ofs1}},{{9.53+ofs2, 4.18635+ofs1}}}};
    static const std::vector<std::vector<double>> SOUTH_FACING_CROSSWALKS = {{{{15.171-ofs1, 3.0317+ofs2}},{{4.6327-ofs1, 7.968+ofs2}}}};
    static const std::vector<std::vector<double>> NORTH_FACING_CROSSWALKS = {{{{0.5943+ofs1, 9.1037-ofs2}},{{5.2924+ofs1, 7.404-ofs2}}}};

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

// 0.015, 0.06783
// 0, 0.0611
// 15.3 15.202
