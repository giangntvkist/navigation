#include "vk_slam3d/graphbasedslam.hpp"
int main(int argc, char **argv) {
    ros::init(argc, argv, "vk_slam3d");
    ROS_INFO("Running vk_slam3d node!");
    ros::NodeHandle nh;
    slam::GraphBasedSlam map(nh);
    return 0;
}