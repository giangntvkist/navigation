#pragma once
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

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

using namespace std;
using namespace message_filters;

struct pf_vector_t {
    double v[3];
};
struct pf_matrix_t {
    double m[3][3];
};
struct pf_sample_t {
    pf_vector_t pose;
    double weight;
};
struct pf_set_sample_t {
    vector<pf_sample_t> samples;
    pf_vector_t mean;
    pf_matrix_t cov;
};
struct pf_scan_t {
    vector<pf_vector_t> ranges;
};

#define occupied 100
#define free 0
#define unknown -1

#define omni 0
#define diff 1
#define empt 0
#define no_empt 1

/** ------------------------------- **/
nav_msgs::OccupancyGrid map_t;
pf_scan_t scan_t;
pf_vector_t odom;
vector<double> map_dist_t;

bool map_data = false, sensor_data = false;
bool first_time = true, error = false;
bool init_localization = false;
bool init_sample = true;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void dataCallback(const nav_msgs::Odometry& msg, const sensor_msgs::LaserScan& scan);
void init_poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
bool map_valid(int idx, int idy, nav_msgs::OccupancyGrid& map);
void lookuptable(nav_msgs::OccupancyGrid& map, vector<double>& map_dist);

void scan_filter(pf_set_sample_t& s, pf_scan_t& scan, nav_msgs::OccupancyGrid& map, pf_scan_t& scan_w, pf_vector_t& laser_pose, vector<double>& map_dist);
double ray_tracing(double x_, double y_, double anpha, nav_msgs::OccupancyGrid& map);
double likelihoodfield_model(pf_scan_t& scan_w, pf_vector_t& q, nav_msgs::OccupancyGrid& map, pf_vector_t& laser_pose, vector<double>& map_dist);
double beams_model(pf_scan_t& scan_w, pf_vector_t& q, nav_msgs::OccupancyGrid& map, pf_vector_t& laser_pose);

double normalize(double z);
double angle_diff(double a, double b);
void mean_covariance(pf_set_sample_t& s);
void uniform_sample(nav_msgs::OccupancyGrid& map, pf_set_sample_t& s);
void normal_sample(nav_msgs::OccupancyGrid& map, pf_set_sample_t& s);
pf_vector_t motion_model(pf_vector_t u_t[2], pf_vector_t q_t_1);
bool update_filter(pf_vector_t u_t[2]);
void update_motion(pf_vector_t u_t[2], pf_set_sample_t& s);

void KLDSamplingMCL(pf_set_sample_t& S_t_1, pf_vector_t u_t[2], pf_scan_t& scan, nav_msgs::OccupancyGrid& map, pf_set_sample_t& S_t, 
    pf_scan_t& scan_w, pf_vector_t& laser_pose, vector<double>& map_dist);
void AugmentedMCL(pf_set_sample_t& S_t_1, pf_vector_t u_t[2], pf_scan_t& scan, nav_msgs::OccupancyGrid& map, pf_set_sample_t& S_t, 
    pf_scan_t& scan_w, pf_vector_t& laser_pose, vector<double>& map_dist);
double computeEntropy(pf_set_sample_t& S_t);
void mcl_publisher(geometry_msgs::PoseArray& pose_arr, geometry_msgs::PoseWithCovarianceStamped& q_t, sensor_msgs::PointCloud& pcl, 
    pf_set_sample_t& s, pf_vector_t laser_pose, pf_scan_t& scan, nav_msgs::OccupancyGrid& map);

