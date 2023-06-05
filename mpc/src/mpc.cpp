#include "mpc/mpc_.hpp"
int main(int argc,char **argv) {
    ROS_INFO("Running model predict controller node!");
    ros::init(argc, argv, "mpc");
    ros::NodeHandle nh;
    MPC mpc_controller(&nh);
    return 0;
}