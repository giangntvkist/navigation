#pragma once
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vector>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <random>

using namespace std;
using namespace message_filters;

#define num_laser 2

struct sm_vector_t {
	double v[3];
};

struct sm_laser_scan{
    sm_vector_t pose;
    vector<sm_vector_t> ranges;
};

double range_min;
double range_max;
double angle_increment;
double angle_min;
double angle_max;
bool inverted_laser;
bool scan_data;

int throttle_scan;
string base_scan_frame;
double publish_frequency;

ros::Publisher point_cloud_pub;
sm_laser_scan scan_t[num_laser];

bool scan_valid(double z) {
    return z < range_max && z >= range_min;
}

void scanCallback(const sensor_msgs::LaserScan& msg1, const sensor_msgs::LaserScan& msg2) {
    float inv_scan;
    if(inverted_laser) {
        inv_scan = -1.0;
    }else {
        inv_scan = 1.0;
    }

    scan_t[0].pose.v[0] = 0.289;
    scan_t[0].pose.v[1] = 0.0;
    scan_t[0].pose.v[2] = 0.0;
    scan_t[0].ranges.clear();
    sm_vector_t ray_i;
    for(int i = 0; i < msg1.ranges.size(); i += throttle_scan) {
        if(scan_valid(msg1.ranges[i])) {
            ray_i.v[0] = msg1.ranges[i];
            ray_i.v[1] = inv_scan*(i*angle_increment - angle_max);
            scan_t[0].ranges.push_back(ray_i);
        } 
    }

    scan_t[1].pose.v[0] = -0.289*sin(M_PI/6);
    scan_t[1].pose.v[1] = 0.289*cos(M_PI/6);
    scan_t[1].pose.v[2] = 2*M_PI/3;
    scan_t[1].ranges.clear();
    for(int i = 0; i < msg2.ranges.size(); i += throttle_scan) {
        if(scan_valid(msg2.ranges[i])) {
            ray_i.v[0] = msg2.ranges[i];
            ray_i.v[1] = inv_scan*(i*angle_increment - angle_max);
            scan_t[1].ranges.push_back(ray_i);
        } 
    }
    scan_data = true;
}

void merge_scan(sm_laser_scan (&scan)[2], sensor_msgs::PointCloud& pcl_t) {
    double r, beam_angle;
    int num_points;
    pcl_t.header.frame_id = base_scan_frame;
    pcl_t.header.stamp = ros::Time::now();
    pcl_t.points.clear();
    geometry_msgs::Point32 p;
    for(int i = 0; i < num_laser; i++) {
        num_points = scan[i].ranges.size();
        for(int j = 0; j < num_points; j++) {
            r = scan[i].ranges[j].v[0];
            beam_angle = scan[i].ranges[j].v[1] + scan[i].pose.v[2];
            p.x = scan[i].pose.v[0] + r*cos(beam_angle);
            p.y = scan[i].pose.v[1] + r*sin(beam_angle);
            pcl_t.points.push_back(p);
        }
    }
}
