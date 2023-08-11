#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/Path.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <vector>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <random>
#include <fstream>
#include <unistd.h>
#include <boost/thread.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>

using namespace std;
using namespace Eigen;
using namespace message_filters;

/* Define vector have 3 elements x, y, theta */
struct sm_vector_t {
	double v[3];
};

struct sm_matrix_t {
	double m[3][3];
};

struct sm_corr_t {
	int i;
	int row;
	int col;
	double dist2;
    double weight;
};

struct sm_point_cloud_t {
    vector<sm_vector_t> pcl;
};


#define occupied 100
#define free 0
#define unknown -1

#define omni 0
#define diff 1
#define inf 1e4
#define un_valid -1

bool data_, map_;
int model_type;

int kernel_size;

nav_msgs::OccupancyGrid occ_map;
sensor_msgs::PointCloud point_cloud_t;
nav_msgs::Odometry odom;

ros::Publisher robot_pose_pub, lase_scan_pub;
ros::Subscriber map_sub;

inline double normalize(double z) {
    return atan2(sin(z), cos(z));
}

inline double angle_diff(double a, double b) {
    double d1, d2;
    a = normalize(a);
    b = normalize(b);
    d1 = a - b;
    d2 = 2*M_PI - fabs(d1);
    if(d1 > 0) d2 *= -1.0;
    if(fabs(d1) < fabs(d2)) {
        return d1;
    }else {
        return d2;
    }
}

bool map_valid(int idx, int idy) {
    return (idx >=0 && idx < occ_map.info.width && idy >= 0 && idy < occ_map.info.height);
}

bool update_filter(sm_vector_t u_t[2]) {
    return fabs(sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2))) > min_trans || fabs(angle_diff(u_t[1].v[2], u_t[0].v[2])) > min_rot;
}

void update_motion(sm_vector_t u_t[2], sm_vector_t& q_t) {
    if(model_type == omni) {
        double delta_trans, delta_rot, delta_bearing;
        delta_trans = sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2));
        delta_rot = angle_diff(u_t[1].v[2], u_t[0].v[2]);
        delta_bearing = angle_diff(atan2(u_t[1].v[1] - u_t[0].v[1], u_t[1].v[0] - u_t[0].v[0]), u_t[0].v[2]) + q_t.v[2];
        /* Update current robot pose */
        q_t.v[0] += delta_trans*cos(delta_bearing);
        q_t.v[1] += delta_trans*sin(delta_bearing);
        q_t.v[2] = normalize(q_t.v[2] + delta_rot);
    }else if(model_type == diff) {
        double delta_trans, delta_rot1, delta_rot2;
        if(sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2)) < 0.01) {
            delta_rot1 = 0.0;
        }else {
            delta_rot1 = angle_diff(atan2(u_t[1].v[1] - u_t[0].v[1], u_t[1].v[0] - u_t[0].v[0]), u_t[0].v[2]);
        }
        delta_trans = sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2));
        delta_rot2 = angle_diff(angle_diff(u_t[1].v[2], u_t[0].v[2]), delta_rot1);
        /* Update current robot pose */
        q_t.v[0] += delta_trans*cos(q_t.v[2] + delta_rot1);
        q_t.v[1] += delta_trans*sin(q_t.v[2] + delta_rot1);
        q_t.v[2] = normalize(q_t.v[2] + delta_rot1 + delta_rot2);
    }
}

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
    map_ = true;
}