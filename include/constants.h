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
    static constexpr double LANE_OFFSET = 0.36;
    static constexpr double INNER_LANE_OFFSET = 0.346765;
    static constexpr double MIN_DIST_TO_CAR = 0.3;
    static constexpr double MAX_CAR_DIST = 3.0;
    static constexpr double SIGN_COOLDOWN = 1.0;
    static constexpr double TOLERANCE_SQUARED = 0.01;
    // static constexpr double STOP_DURATION = 1.50;
    // static constexpr double NORMAL_SPEED = 0.175;
    // static constexpr double FAST_SPEED = 0.4;
    static constexpr double SOFT_MAX_STEERING = 0.42 * 180 / M_PI;
    static constexpr double HARD_MAX_STEERING = 25.0;

    // parking coordinates
    static constexpr std::array<double, 2> PARKING_SPOT_RIGHT = {9.50, 0.372};
    static constexpr std::array<double, 2> PARKING_SPOT_LEFT = {9.50, 1.110};
    static constexpr std::array<double, 2> PARKING_SIGN_TO_CAR1 = {0.4592, -0.073};

    static constexpr double ofs6 = 0.148673;
    static constexpr double hsw = 0.0225;
    static constexpr double haha = 369. - INNER_LANE_OFFSET;
    // add half of inner lane width to the x values
    // static constexpr std::array<double, 13> Y_ALIGNED_LANE_CENTERS = {0.22237, 0.591617, 2.383851, 2.754291, 4.63, 4.9981, 6.49, 6.864, 15.17, 16.963, 15.54, 15.7404, 16.112};
    static const std::vector<double> NORTH_FACING_LANE_CENTERS = {0.591617+ofs6, 2.754291+ofs6, 4.9981+ofs6, 6.49+ofs6, 6.864+ofs6, 16.963+ofs6, 15.54+ofs6, 16.112+ofs6};
    static const std::vector<double> SOUTH_FACING_LANE_CENTERS = {0.22237+ofs6, 2.383851+ofs6, 4.63+ofs6, 15.17+ofs6, 15.7404+ofs6};
    // add half of inner lane width to the y values
    // static constexpr std::array<double, 13> X_ALIGNED_LANE_CENTERS = {13.314624, 12.94356, 10.669, 10.2963, 3.89, 0.598716, 0.9698, 3.516515, 3.88667, 6.4122, 6.78514, 11.6955, 12.0661};
    static const std::vector<double> EAST_FACING_LANE_CENTERS = {12.94356+ofs6, 10.2963+ofs6, 0.598716+ofs6, 3.516515+ofs6, 6.4122+ofs6, 11.6955+ofs6};
    static const std::vector<double> WEST_FACING_LANE_CENTERS = {13.314624+ofs6, 10.669+ofs6, 3.89+ofs6, 0.9698+ofs6, 3.88667+ofs6, 6.78514+ofs6, 12.0661+ofs6};

    
    // intersection coordinates
    static const std::vector<std::vector<double>> SOUTH_FACING_INTERSECTIONS = {{{16.0362-ofs6, 11.7209-hsw}},{{15.465-ofs6, 4.6082-hsw}},{{15.17155+ofs6, 1.313784-hsw}},{{4.929-ofs6, 4.603335-hsw}},{{4.929-ofs6, 1.68375-hsw}},{{2.681-ofs6, 1.68375-hsw}},{{2.681-ofs6, 4.603335-hsw}},{{0.520-ofs6, 4.6-hsw}},{{0.520-ofs6, 7.49854-hsw}},{{0.519-ofs6, 10.613252-hsw}}};
    static const std::vector<std::vector<double>> NORTH_FACING_INTERSECTIONS = {{{0.5943+ofs6, 9.1037+hsw}},{{0.5943+ofs6, 5.9901+hsw}},{{0.5943+ofs6, 3.0968+hsw}},{{2.75684+ofs6, 3.0964+hsw}},{{2.75684+ofs6, 5.99+hsw}},{{5.00+ofs6, 5.99+hsw}},{{5.00+ofs6, 3.096+hsw}},{{6.79-ofs6, 3.467+hsw}},{{16.115+ofs6, 9.5346+hsw}}};
    static const std::vector<std::vector<double>> WEST_FACING_INTERSECTIONS = {{{17.16466-hsw, 10.663231+ofs6}},{{1.31-hsw, 6.78+ofs6}},{{3.4726-hsw, 6.782+ofs6}},{{5.7189-hsw, 6.7818+ofs6}},{{5.7208-hsw, 3.88664+ofs6}},{{3.472454-hsw, 3.88726+ofs6}},{{1.3104-hsw, 3.887124+ofs6}},{{7.582-hsw, 3.886314+ofs6}},{{3.474682-hsw, 0.9671+ofs6}},{{16.26-hsw, 3.889+ofs6}}};
    static const std::vector<std::vector<double>> EAST_FACING_INTERSECTIONS = {{{1.9637+hsw, 0.89432-ofs6}}, {{1.96576+hsw, 3.8132-ofs6}}, {{1.9647+hsw, 6.7074-ofs6}}, {{4.2125+hsw, 6.707-ofs6}}, {{4.2107+hsw, 3.813357-ofs6}}, {{4.2132+hsw, 0.8932-ofs6}}, {{6.0735+hsw, 0.5949+ofs6}}, {{14.7516+hsw, 0.89253-ofs6}}, {{14.99+hsw, 10.5895-ofs6}}};

    static constexpr double ofs7 = 0.445;
    static constexpr double ofs3 = 0.062;
    static const std::vector<std::vector<double>> SOUTH_FACING_SIGNS = {
        {{15.015, 4.6702}},
        {{15.015, 1.375784}},
        {{4.482, 4.665335}},
        {{4.482, 1.74575}},
        {{2.234, 1.74575}},
        {{2.234, 4.665335}},
        {{0.073, 4.662}},
        {{0.073, 7.56054}},
        {{0.073, 10.67525}}
    };
    // static const std::vector<std::vector<double>> NORTH_FACING_SIGNS = {{{0.5943+ofs7, 9.1037-ofs3}},{{0.5943+ofs7, 5.9901-ofs3}},{{0.5943+ofs7, 3.0968-ofs3}},{{2.75684+ofs7, 3.0964-ofs3}},{{2.75684+ofs7, 5.99-ofs3}},{{5.00+ofs7, 5.99-ofs3}},{{5.00+ofs7, 3.096-ofs3}},{{6.79-2*ofs6+ofs7, 3.467-ofs3}}};
    static const std::vector<std::vector<double>> NORTH_FACING_SIGNS = {
        {{1.0393, 9.0417}},
        {{1.0393, 5.9281}},
        {{1.0393, 3.0348}},
        {{3.2018, 3.0344}},
        {{3.2018, 5.9280}},
        {{5.4450, 5.9280}},
        {{5.4450, 3.0340}},
        {{6.9377, 3.4050}}
    };
    // static const std::vector<std::vector<double>> WEST_FACING_SIGNS = {{{1.31+ofs3, 6.78+ofs7}},{{3.4726+ofs3, 6.782+ofs7}},{{5.7189+ofs3, 6.7818+ofs7}},{{5.7208+ofs3, 3.88664+ofs7}},{{3.472454+ofs3, 3.88726+ofs7}},{{1.3104+ofs3, 3.887124+ofs7}},{{7.582+ofs3, 3.886314+ofs7}},{{3.474682+ofs3, 0.9671+ofs7}},{{16.26+ofs3, 3.889+ofs7}}};
    static const std::vector<std::vector<double>> WEST_FACING_SIGNS = {
        {{1.3720, 7.2250}},
        {{3.5346, 7.2270}},
        {{5.7809, 7.2268}},
        {{5.7828, 4.3316}},
        {{3.5345, 4.3323}},
        {{1.3724, 4.3321}},
        {{7.6440, 4.3313}},
        {{3.5367, 1.4121}},
        {{16.3220, 4.3340}}
    };
    // static const std::vector<std::vector<double>> EAST_FACING_SIGNS = {{{1.9637-ofs3, 0.89432-ofs7}}, {{1.96576-ofs3, 3.8132-ofs7}}, {{1.9647-ofs3, 6.7074-ofs7}}, {{4.2125-ofs3, 6.707-ofs7}}, {{4.2107-ofs3, 3.813357-ofs7}}, {{4.2132-ofs3, 0.8932-ofs7}}, {{6.0735-ofs3, 0.5949+ofs6*2-ofs7}}, {{14.7516-ofs3, 0.89253-ofs7}}};
    static const std::vector<std::vector<double>> EAST_FACING_SIGNS = {
        {{1.9017, 0.4493}},
        {{1.9038, 3.3682}},
        {{1.9027, 6.2624}},
        {{4.1505, 6.2620}},
        {{4.1487, 3.3684}},
        {{4.1512, 0.4482}},
        {{6.0115, 0.4472}},
        {{14.6896, 0.4475}}
    };
    // object coordinates
    static const std::vector<std::vector<double>> PARKING_SIGN_POSES = {{{{8.922497, 0.487}}, {{8.922405, 1.008329}}}};
    static const std::vector<std::vector<double>> ROUNDABOUT_POSES = {{{{14.906, 10.190544}}, {{16.5132, 9.455325}}, {{17.247, 11.067}}, {{15.639, 11.80325}}}};
    // static constexpr std::array<std::array<double, 2>, 11> INTERSECTION_SIGN_POSES = {{{{0.067568, 4.669999}}, {{1.90, 0.444024}}, {{3.207346, 3.032455}}, {{3.206, 5.9274}}, {{4.146563, 6.259}}, {{0.072, 10.682}}, {{15.016, 4.67363}}, {{7.6468, 4.33}}, {{5.791, 4.33}}, {{4.4838, 1.752361}}, {{6.06227, 0.511846}}}};
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
