#include "battery/battery.hpp"

int main(int argc,char **argv) {
    ROS_INFO("Running battery node!");
    ros::init(argc, argv, "battery");
    ros::NodeHandle nh;
    BATTERY battery_com(&nh);
    return 0;
}