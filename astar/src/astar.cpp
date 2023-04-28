#include "astar/astar_.hpp"
int main(int argc,char **argv) {
    ROS_INFO("Running A star path planner node!");
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("astar", 10);
    
    ros::Subscriber map_sub = nh.subscribe("cost_map", 10, mapCallback);
    ros::Subscriber initpose_sub = nh.subscribe("init_pose", 10, initposeCallback);
    ros::Subscriber goalpose_sub = nh.subscribe("move_base_simple/goal", 10, goalposeCallback);

    return 0;
}