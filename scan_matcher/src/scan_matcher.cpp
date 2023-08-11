#include "scan_matcher/scan_matcher.hpp"
int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_fusion");
    ROS_INFO("Running sensor fusion node!");
    ros::NodeHandle nh;

    robot_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot_pose", 1);
    lase_scan_pub = nh.advertise<sensor_msgs::PointCloud>("point_cloud", 1);

    map_sub = nh.subscribe("map", 10, mapCallback);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud> scan_sub(nh, "scan", 10);

    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, scan_sub);
    sync.registerCallback(dataCallback);

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