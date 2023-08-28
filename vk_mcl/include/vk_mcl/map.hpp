#pragma once
#include "vk_mcl/vk_mcl.hpp"
#include "vk_mcl/parameters.hpp"

/** Subscribe occupancy grid map */
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_t.info.width = msg->info.width;
    map_t.info.height = msg->info.height;
    map_t.info.resolution = msg->info.resolution;
    map_t.info.origin.position.x = msg->info.origin.position.x;
    map_t.info.origin.position.y = msg->info.origin.position.y;
    map_t.data.clear();
    for(int i = 0; i < msg->data.size(); i++) {
        map_t.data.push_back(msg->data[i]);
    }
    map_data = true;
}

/** Subscribe sensor data: odometry and laser scan */
void dataCallback(const nav_msgs::Odometry& msg, const sensor_msgs::LaserScan& scan) {
    odom.v[0] = msg.pose.pose.position.x;
    odom.v[1] = msg.pose.pose.position.y;
    odom.v[2] = tf::getYaw(msg.pose.pose.orientation);

    float inv_scan;
    if(inverse_laser) {
        inv_scan = -1.0;
    }else {
        inv_scan = 1.0;
    }
    scan_t.ranges.clear();
    pf_vector_t r;
    for(int i = 0; i < max_beams; i += throttle_scan) {
        r.v[0] = scan.ranges[i];
        r.v[1] = inv_scan*(i*angle_increment + angle_min);
        scan_t.ranges.push_back(r);
    }
    sensor_data = true;
}

/** Subscribe initial robot pose*/
void init_poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
    init_pose_x = msg.pose.pose.position.x;
    init_pose_y = msg.pose.pose.position.y;
    init_pose_theta = tf::getYaw(msg.pose.pose.orientation);
    init_sample = true;
}

bool map_valid(int idx, int idy, nav_msgs::OccupancyGrid& map) {
    return (idx >=0 && idx < map.info.width && idy >= 0 && idy < map.info.height);
}

/** Lookup table for nearest cell-occupied distance */
void lookuptable(nav_msgs::OccupancyGrid& map, vector<double>& map_dist) {
    ROS_INFO("Waiting for get the lookup table...");
    ifstream lookup_table_file;
    lookup_table_file.open(lookup_table_path);
    map_dist.clear();
    double d_min;
    if(lookup_table_file.fail()) {
        ROS_ERROR("Invalid lookup table link!");
        error = true;
    }else {
        for(int i = 0; i < map.data.size(); i++) {
            lookup_table_file >> d_min;
            map_dist.push_back(d_min);
        }
        lookup_table_file.close();
        ROS_INFO("Lookup table complete!");
    }   
}

