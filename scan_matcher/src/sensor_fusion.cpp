#include "scan_matcher/sensor_fusion.hpp"
int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_fusion");
    ROS_INFO("Running sensor fusion node!");
    ros::NodeHandle nh;

    point_cloud_pub = nh.advertise<sensor_msgs::PointCloud>("scan", 1);

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub1(nh, "scan1", 10);
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub2(nh, "scan2", 10);

    typedef sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), laser_scan_sub1, laser_scan_sub2);
    sync.registerCallback(scanCallback);

    if(!ros::param::get("~range_min", range_min)) range_min = 0.05;
    if(!ros::param::get("~range_max", range_max)) range_max = 25.0;
    if(!ros::param::get("~angle_min", angle_min)) angle_min = -M_PI/2;
    if(!ros::param::get("~angle_max", angle_max)) angle_max = M_PI/2;
    if(!ros::param::get("~angle_increment", angle_increment)) angle_increment = M_PI/(3*180);

    if(!ros::param::get("~throttle_scan", throttle_scan)) throttle_scan = 1;
    if(!ros::param::get("~inverted_laser", inverted_laser)) inverted_laser = true;
    if(!ros::param::get("~publish_frequency", publish_frequency)) publish_frequency = 15;

    if(!ros::param::get("~base_scan_frame", base_scan_frame)) base_scan_frame = "base_scan";
    sensor_msgs::PointCloud pcl_t;
    ros::Rate rate(publish_frequency);
    while(ros::ok()) {
        ros::spinOnce();
        if(scan_data) {
            merge_scan(scan_t, pcl_t);
            point_cloud_pub.publish(pcl_t);
        }
        rate.sleep();
    }
    return 0;
}