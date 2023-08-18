#include "scan_matcher/scan_matcher.hpp"
int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_fusion");
    ROS_INFO("Running sensor fusion node!");
    ros::NodeHandle nh;

    robot_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_pose", 1);
    laser_scan_pub = nh.advertise<sensor_msgs::PointCloud>("sm_point_cloud", 1);

    map_sub = nh.subscribe("map", 10, mapCallback);
    amcl_pose_sub = nh.subscribe("amcl_pose", 10, poseCallback);
    point_cloud_sub = nh.subscribe("point_cloud", 1, pointcloud_Callback);

    if(!ros::param::get("~publish_frequency", publish_frequency)) publish_frequency = 10;

    ros::Rate rate(publish_frequency);
    while(ros::ok()) {
        ros::spinOnce();
        if(data_) {
            
        }
        rate.sleep();
    }
    return 0;
}