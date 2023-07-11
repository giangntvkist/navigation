#include "vk_slam/vk_slam.hpp"
#include "vk_slam/optimization.cpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "vk_slam");
    ROS_INFO("Running vk_slam node!");
    ros::NodeHandle nh;

    // ros::Rate rate(map_update_interval);
    // while(ros::ok()) {
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    return 0;
}