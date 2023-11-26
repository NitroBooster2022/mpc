#include <ros/ros.h>
#include "utility.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "utility_node");
    ros::NodeHandle nh;
    Utility utility(nh);
    ros::spin();
    return 0;
}