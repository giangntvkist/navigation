#pragma once
#include "mcl/mcl.hpp"

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    occ_map.info.width = msg->info.width;
    occ_map.info.height = msg->info.height;
    occ_map.info.resolution = msg->info.resolution;
    occ_map.info.origin.position.x = msg->info.origin.position.x;
    occ_map.info.origin.position.y = msg->info.origin.position.y;
    occ_map.data.clear();
    for(int i = 0; i < msg->data.size(); i++) {
        occ_map.data.push_back(msg->data[i]);
    }
    _map = true;
}

void dataCallback(const nav_msgs::Odometry& msg, const sensor_msgs::LaserScan& scan) {
    laser_scan.angle_min = scan.angle_min;
    laser_scan.angle_max = scan.angle_max;
    laser_scan.angle_increment = scan.angle_increment;
    laser_scan.range_min = scan.range_min;
    laser_scan.range_max = laser_scan.range_max;
    laser_scan.ranges.clear();
    for(int i = 0; i < scan.ranges.size(); i += throttle_scan) {
        laser_scan.ranges.push_back(scan.ranges[i]);
    }
    odom.pose.pose.position.x = msg.pose.pose.position.x;
	odom.pose.pose.position.y = msg.pose.pose.position.y;
	odom.pose.pose.orientation = msg.pose.pose.orientation;
    _data = true;
}

bool map_valid(int idx, int idy) {
    return (idx >=0 && idx < occ_map.info.width && idy >= 0 && idy < occ_map.info.height);
}

void get_lookuptable(nav_msgs::OccupancyGrid& map) {
    ROS_INFO("Waiting for get the lookup table...");
    if(lookup_table) {
        int idx, idy;
        double d_min;
        int kernel_size = max_occ_dist/map.info.resolution;
        set_occ_dist.clear();
        set_occ_dist.resize(map.data.size());
        for(int i = 0; i < map.data.size(); i++) {
            cout << "\r";
            cout << "Loading the lookup table: " << int((i+1)*100/map.data.size()) << "% ...";
            if(map.data[i] != occupied) {
                set_occ_dist[i] = max_occ_dist;
            }else {
                idy = i/map.info.width;
                idx = i - idy*map.info.width;
                for(int j = idy - kernel_size; j <= idy + kernel_size; j++) {
                    if(j >= 0 && j < map.info.height) {
                        for(int k = idx - kernel_size; k <= idx + kernel_size; k++) {
                            if(k >= 0 && k < map.info.width) {
                                if(map.data[j*map.info.width + k] != occupied) {
                                    d_min = map.info.resolution*sqrt(pow(idx - k, 2) + pow(idy - j, 2));
                                }else {
                                    d_min = 0.0;
                                }
                                set_occ_dist[j*map.info.width + k] = std::min(d_min, set_occ_dist[j*map.info.width + k]);
                            }
                        }
                    }
                }
            }
        }
        ofstream lookup_table_file(lookup_table_path, std::ofstream::out | std::ofstream::trunc);
        if(lookup_table_file.fail()) {
            ROS_ERROR("Invalid lookup table link!");
            error_ = true;
        }else {
            for(int i = 0; i < map.info.height; i++) {
                for(int j = 0; j < map.info.width; j++) {
                    lookup_table_file << set_occ_dist[i*map.info.width+j] << " ";
                }
                lookup_table_file << endl;
            }
        }
        lookup_table_file.close();
        ROS_INFO("Got the lookup table!");
    }else {
        ifstream lookup_table_file;
        lookup_table_file.open(lookup_table_path);
        set_occ_dist.clear();
        double d_min;
        if(lookup_table_file.fail()) {
            ROS_ERROR("Invalid lookup table link!");
            error_ = true;
        }else {
            for(int i = 0; i < map.data.size(); i++) {
                lookup_table_file >> d_min;
                set_occ_dist.push_back(d_min);
            }
            lookup_table_file.close();
            ROS_INFO("Got the lookup table!");
        }   
    }
}