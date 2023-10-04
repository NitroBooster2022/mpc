#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "utils/Telemetry.h"
#include <fstream>
#include <vector>
#include <sstream>

class MPCNode {
public:
    MPCNode();
    ~MPCNode() = default;

    void run();

private:
    void readWaypoints(const std::string& filename);
    void telemetryCallback(const utils::Telemetry::ConstPtr& msg);

    // For converting back and forth between radians and degrees.
    constexpr double pi();
    double deg2rad(double x);
    double rad2deg(double x);
    // Evaluate a polynomial.
    double polyeval(Eigen::VectorXd coeffs, double x);
    Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                            int order);
    // convert from map coordinate to car coordinates
    void map2car(double px, double py, double psi, const vector<double>& ptsx_map, const vector<double>& ptsy_map,
                Eigen::VectorXd & ptsx_car, Eigen::VectorXd & ptsy_car);
    bool visualize;
    bool static_path;
    double px;
    double py;
    double psi;
    double v;
    double steer_angle; 
    double throttle_value;
    std::vector<std::pair<double, double>> waypoints;
    MPC mpc;
    
    // ros::Publisher steer_pub;
    // ros::Publisher throttle_pub;
    ros::Publisher cmd_vel_pub;
    ros::Publisher mpc_trajectory_pub;
    ros::Publisher ref_trajectory_pub;
};

MPCNode::MPCNode()
    : visualize(true),
      static_path(false) 
{
    ros::NodeHandle nh;
    
    // Initialize ROS publishers
    // steer_pub = nh.advertise<std_msgs::Float64>("steer_cmd", 1);
    // throttle_pub = nh.advertise<std_msgs::Float64>("throttle_cmd", 1);
    cmd_vel_pub = nh.advertise<std_msgs::String>("/automobile/command", 1);

    if (visualize) {
        mpc_trajectory_pub = nh.advertise<nav_msgs::Path>("mpc_trajectory", 1);
        ref_trajectory_pub = nh.advertise<nav_msgs::Path>("ref_trajectory", 1);
    }

    if (static_path) {
        readWaypoints("waypoints.txt");
    }
}

void MPCNode::readWaypoints(const std::string& filename) {
    std::cout << "Reading waypoints from " << filename << std::endl;
    std::ifstream file(filename);
    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        double x, y;
        ss >> x;
        if (ss.peek() == ',') {
            ss.ignore();
        }
        ss >> y;
        waypoints.emplace_back(x, y);
    }
    std::cout << "Read " << waypoints.size() << " waypoints." << std::endl;
}

void MPCNode::telemetryCallback(const utils::Telemetry::ConstPtr& msg) {
    std::vector<double> ptsx;
    std::vector<double> ptsy;

    if(static_path) {
        ptsx.resize(waypoints.size());
        ptsy.resize(waypoints.size());
        for (size_t i = 0; i < 3; ++i) {
            ptsx[i] = waypoints[i].first;
            ptsy[i] = waypoints[i].second;
        }
    } else {
        ptsx = msg->ptsx;
        ptsy = msg->ptsy;
    }
    
    px = msg->x;
    py = msg->y;
    psi = msg->psi;
    v = msg->speed;
    steer_angle = deg2rad(msg->steering_angle); 
    
    // print out the state, keep 2 decimal places
    ROS_INFO("px: %.2f, py: %.2f, psi: %.2f, v: %.2f, steer_angle: %.2f, throttle: %.2f", px, py, psi, v, steer_angle, msg->throttle);
    throttle_value = msg->throttle;
    
    /*
    * Calculate steeering angle and throttle using MPC.
    */
    // convert from map coordinate to car coordinate
    Eigen::VectorXd ptsx_car(ptsx.size());
    Eigen::VectorXd ptsy_car(ptsy.size());
    
    // print ptsx and ptsy
    // for (size_t i = 0; i < ptsx.size(); ++i) {
    //     ROS_INFO("ptsx: %.2f, ptsy: %.2f", ptsx[i], ptsy[i]);
    // }
    map2car(px, py, psi, ptsx, ptsy, ptsx_car, ptsy_car);

    // compute the coefficients
    auto coeffs = polyfit(ptsx_car, ptsy_car, 3); // 3rd order line fitting

    // state in car coordniates
    Eigen::VectorXd state(6); // {x, y, psi, v, cte, epsi}

    // add latency 100ms
    double latency = 0.65;
    double Lf = 0.27;
    // v *= 0.44704;                             // convert from mph to m/s
    px = 0 + v * cos(0) * latency;            // px:  px0 = 0, due to the car coordinate system
    py = 0 + v * sin(0) * latency;;           // py:  psi=0 and y is point to the left of the car
    psi = 0 - v / Lf * steer_angle * latency;   // psi:  psi0 = 0, due to the car coordinate system
    double epsi = 0 - atan(coeffs[1]) - v / Lf * steer_angle * latency; // epsi is the error of psi relative to the reference line
    double cte = polyeval(coeffs, 0) - 0 + v * sin(0- atan(coeffs[1])) * latency; // cte is the error of y relative to the reference line
    double original_speed = v;
    v += throttle_value * latency;

    // print epsi and cte
    ROS_INFO("cte: %.2f, epsi: %.2f", cte, epsi);
    state << px, py, psi, v, cte, epsi;

//          double latency = 0.1;
//          double Lf = 2.67;
//          v *= 0.44704;                             // convert from mph to m/s
//          px = 0 + v * cos(steer_angle) * latency;  // px:  px0 = 0, due to the car coordinate system
//          py = 0 + v * sin(steer_angle) * latency;  // py:  py0 = 0, due to the car coordinate system
//          psi = - v / Lf * steer_angle * latency;   // psi:  psi0 = 0, due to the car coordinate system
//          double cte = polyeval(coeffs, px);
//          double epsi = atan(coeffs[1]+2*coeffs[2]*px + 3*coeffs[3]*px*px);
//          state << px, py, psi, v, cte, epsi;

    // call MPC solver
    auto vars = mpc.Solve(state, coeffs);


    // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
    // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
    // steering angle = - psi
    double steer_value;
    double acceleration_value;

    steer_value = -rad2deg(vars[0]);
    acceleration_value = vars[1];

    double speed_command = original_speed + acceleration_value * latency;
    ROS_INFO("steer: %.2f, accel: %.2f, speed: %.2f", steer_value, acceleration_value, speed_command);

    // std_msgs::Float64 steer_msg;
    // steer_msg.data = steer_value;  // steer_value computed as in original code
    // steer_pub.publish(steer_msg);

    // std_msgs::Float64 throttle_msg;
    // throttle_msg.data = throttle_value;  // throttle_value computed as in original code
    // throttle_pub.publish(throttle_msg);
    std_msgs::String cmd_vel_msg;
    std::stringstream ss1;
    std::stringstream ss2;

    // convert this string from python format to c++: '{"action":"2","steerAngle":'+str(float(steering_angle))+'}'
    ss1 << "{\"action\":\"2\",\"steerAngle\":" << steer_value << "}";
    ss2 << "{\"action\":\"1\",\"speed\":" << speed_command << "}";

    cmd_vel_msg.data = ss1.str();
    cmd_vel_pub.publish(cmd_vel_msg);
    cmd_vel_msg.data = ss2.str();
    cmd_vel_pub.publish(cmd_vel_msg);

    if(visualize) {
        // Create and populate nav_msgs::Path messages for the MPC and reference trajectories
        nav_msgs::Path mpc_path;
        mpc_path.header.stamp = ros::Time::now();
        mpc_path.header.frame_id = "odom";  // Change the frame_id as per your setup
        
        for(size_t i = 2; i < vars.size(); i += 2) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "odom";  // Change the frame_id as per your setup
            pose.pose.position.x = vars[i];
            pose.pose.position.y = vars[i+1];
            pose.pose.orientation.w = 1.0;
            mpc_path.poses.push_back(pose);
        }
        
        nav_msgs::Path ref_path;
        ref_path.header.stamp = ros::Time::now();
        ref_path.header.frame_id = "odom";  // Change the frame_id as per your setup

        for(int i = 1; i < ptsx_car.size(); ++i) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "odom";  // Change the frame_id as per your setup
            pose.pose.position.x = ptsx_car[i];
            pose.pose.position.y = ptsy_car[i];
            pose.pose.orientation.w = 1.0;
            ref_path.poses.push_back(pose);
        }

        // Publish the path messages
        mpc_trajectory_pub.publish(mpc_path);
        ref_trajectory_pub.publish(ref_path);
    }

}

// For converting back and forth between radians and degrees.
constexpr double MPCNode::pi() { return M_PI; }
double MPCNode::deg2rad(double x) { return x * pi() / 180; }
double MPCNode::rad2deg(double x) { return x * 180 / pi(); }
// Evaluate a polynomial.
double MPCNode::polyeval(Eigen::VectorXd coeffs, double x) {
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++) {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd MPCNode::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
        for (int i = 0; i < order; i++) {
        A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

// convert from map coordinate to car coordinates
void MPCNode::map2car(double px, double py, double psi, const vector<double>& ptsx_map, const vector<double>& ptsy_map,
            Eigen::VectorXd & ptsx_car, Eigen::VectorXd & ptsy_car){

    for(size_t i=0; i< ptsx_map.size(); i++){
        double dx = ptsx_map[i] - px;
        double dy = ptsy_map[i] - py;
        ptsx_car[i] = dx * cos(-psi) - dy * sin(-psi);
        ptsy_car[i] = dx * sin(-psi) + dy * cos(-psi);
    }
}
    
void MPCNode::run() {
    ros::NodeHandle nh;
    ros::Subscriber telemetry_sub = nh.subscribe<utils::Telemetry>("telemetry", 1, &MPCNode::telemetryCallback, this);
    ros::spin();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mpc_node");

    MPCNode mpc_node;
    mpc_node.run();

    return 0;
}
