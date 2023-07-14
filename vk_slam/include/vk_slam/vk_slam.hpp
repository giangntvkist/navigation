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

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>

using namespace std;
using namespace Eigen;
using namespace message_filters;

/* Define vector have 3 elements x, y, theta */
struct sl_vector_t {
	double v[3];
};

/* Define covariance matrix */
struct sl_matrix_t {
	double m[3][3];
};

/* Define laser scan data: distance data & ray angle*/
struct sl_scan_t {
	vector<sl_vector_t> ranges;
};

/* Define node: pose & laser scan */
struct sl_node_t {
	sl_vector_t pose;
	sl_scan_t scan;
	int idx;
};

/* Define edge */
struct sl_edge_t {
	int i;
	int j;
	sl_vector_t z;
	sl_matrix_t inv_cov;
	int idx;
};

/* Define graph */
struct sl_graph_t {
	vector<sl_node_t> set_node_t;
	vector<sl_edge_t> set_edge_t;
	double cost_value;
};

#define inf 1e4

double range_min;
double range_max;
double angle_increment;
double angle_min;
double angle_max;

double laser_pose_x;
double laser_pose_y;
double laser_pose_theta;

int throttle_scan;

string map_frame;
string base_frame;
string odom_frame;

bool inverted_laser;
bool loop_closure_detected;

int max_inter = 1e3;
double e_converged = 1e-3;
double map_update_interval = 10;

nav_msgs::Odometry odom;
sensor_msgs::LaserScan laser_scan;
nav_msgs::OccupancyGrid occ_map;

sl_graph_t graph_t;

double normalize(double z);
double angle_diff(double a, double b);

// Eigen::Matrix3d inverse_covariance_func(...);
Eigen::Vector3d error_func(Eigen::Vector3d& x_i, Eigen::Vector3d& x_j, Eigen::Vector3d& z_ij);
Eigen::Matrix<double, 3, 6> jacobian_func(Eigen::Vector3d& x_i, Eigen::Vector3d& x_j, Eigen::Vector3d& z_ij);
double cost_func(vector<sl_edge_t>& set_edge_t_, Eigen::VectorXd& x);
void optimization(sl_graph_t& graph_t_);
void pose_graph_test(sl_graph_t& graph_t_);