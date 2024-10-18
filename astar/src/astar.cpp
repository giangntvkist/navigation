#include "astar/astar_.hpp"
int main(int argc,char **argv) {
    ROS_INFO("Running A star path planner node!");
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    Astar path_planner(&nh);
    return 0;
}