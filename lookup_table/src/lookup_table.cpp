#include <ros/ros.h>
#include <ros/console.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include <vector>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <random>
#include <fstream>
#include <unistd.h>

using namespace std;

#define occupied 100
#define free 0
#define unknown -1

vector<double> distance_map;
string lookup_table_path;
nav_msgs::OccupancyGrid map_t;
bool map_data;
double max_occ_dist;

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

void lookuptable(nav_msgs::OccupancyGrid& map) {
    ROS_INFO("Waiting for get the lookup table...");
    int idx, idy;
    double d_min;
    int kernel_size = max_occ_dist/map.info.resolution;
    distance_map.clear();
    distance_map.resize(map.data.size());
    int map_size = map.data.size();
    for(int i = 0; i < map.data.size(); i++) {
        cout << "\r";
        cout << "Loading the lookup table: " << int((i+1)*100/map_size) << "% ...";
        if(map.data[i] != occupied) {
            distance_map[i] = max_occ_dist;
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
                            distance_map[j*map.info.width + k] = std::min(d_min, distance_map[j*map.info.width + k]);
                        }
                    }
                }
            }
        }
    }
    ofstream lookup_table_file(lookup_table_path, std::ofstream::out | std::ofstream::trunc);
    if(lookup_table_file.fail()) {
        ROS_ERROR("Invalid lookup table link!");
    }else {
        for(int i = 0; i < map.info.height; i++) {
            for(int j = 0; j < map.info.width; j++) {
                lookup_table_file << distance_map[i*map.info.width+j] << " ";
            }
            lookup_table_file << endl;
        }
        lookup_table_file.close();
        ROS_INFO("Lookup table complete!");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lookup_table");
    ROS_INFO("Running lookup table node!");
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("map", 1, mapCallback);
    if(!ros::param::get("~max_occ_dist", max_occ_dist)) max_occ_dist = 2.0;
    if(!ros::param::get("~lookup_table_path", lookup_table_path)) lookup_table_path = "";

    while(ros::ok()) {
        ros::spinOnce();
        if(map_data) {
            lookuptable(map_t);
            break;
        }
    }
    return 0;
}
