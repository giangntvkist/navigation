#include "vk_slam/vk_slam.hpp"
#include "vk_slam/observation.cpp"
#include "vk_slam/optimization.cpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "vk_slam");
    ROS_INFO("Running vk_slam node!");
    ros::NodeHandle nh;

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 10);
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub(nh, "scan1", 10);
    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),odom_sub, laser_scan_sub);
    sync.registerCallback(dataCallback);

    if(!ros::param::get("~range_min", range_min)) range_min = 0.05;
    if(!ros::param::get("~range_max", range_max)) range_max = 25.0;
    if(!ros::param::get("~angle_min", angle_min)) angle_min = -M_PI/2;
    if(!ros::param::get("~angle_max", angle_max)) angle_max = M_PI/2;
    if(!ros::param::get("~angle_increment", angle_increment)) angle_increment = M_PI/(3*180);

    if(!ros::param::get("~laser_pose_x", laser_pose_x)) laser_pose_x = 0.0;
    if(!ros::param::get("~laser_pose_y", laser_pose_y)) laser_pose_y = -0.0;
    if(!ros::param::get("~laser_pose_theta", laser_pose_theta)) laser_pose_theta = 0.0;
    if(!ros::param::get("~throttle_scan", throttle_scan)) throttle_scan = 2;
    if(!ros::param::get("~inverted_laser", inverted_laser)) inverted_laser = true;

    if(!ros::param::get("~max_inter", max_inter)) max_inter = 1e3;
    if(!ros::param::get("~converged_graph", converged_graph)) converged_graph = 1e-3;
    if(!ros::param::get("~map_update_interval", map_update_interval)) map_update_interval = 20;

    if(!ros::param::get("~min_trans", min_trans)) min_trans = 0.5;
    if(!ros::param::get("~min_rot", min_rot)) min_rot = 0.5;

    if(!ros::param::get("~base_frame", base_frame)) base_frame = "base_link";
    if(!ros::param::get("~map_frame", map_frame)) map_frame = "map";

    ros::Rate rate(map_update_interval);
    while(ros::ok()) {
        ros::spinOnce();
        
        rate.sleep();
    }
    return 0;
}