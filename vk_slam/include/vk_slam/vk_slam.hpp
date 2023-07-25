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
#include <eigen3/Eigen/SVD>
#include <eigen3/Eigen/SparseCholesky>
#include <eigen3/Eigen/SparseQR>

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
    sl_matrix_t inv_cov;
};

/* Define edge */
struct sl_edge_t {
	int i;
	int j;
	sl_vector_t z;
	sl_matrix_t inv_cov;
};

/* Define graph */
struct sl_graph_t {
	vector<sl_node_t> set_node_t;
	vector<sl_edge_t> set_edge_t;
};

#define occupied 100
#define free 0
#define unknown -1

#define l_0 0.0
#define l_occ 1.0986
#define l_free -1.0986

#define omni 0
#define diff 1
#define inf 1e4
int model_type; /* Type of model robot: differential or omnidirectional robot */

double range_min;
double range_max;
double angle_increment;
double angle_min;
double angle_max;

double laser_pose_x;
double laser_pose_y;
double laser_pose_theta;

double init_pose_x;
double init_pose_y;
double init_pose_theta;

int throttle_scan;

string map_frame;
string base_frame;
string odom_frame;

bool data_ = false;
bool first_time = true;
bool inverted_laser;
bool loop_closure_detected;
bool max_inter_ICP;

int max_inter;
double converged_graph;
double map_update_interval;
double sigma;
double loop_kernel_size;

double min_trans, min_rot;
int map_width, map_height;
double map_resolution;
double dist_threshold;
double min_cumulative_distance;
double e_threshold;
bool scan_matching_success;

double delta_x, delta_y, delta_theta; /* Using for calculating approximate Hessian matrix */

ros::Publisher map_pub, pose_graph_pub;

nav_msgs::Odometry odom;
sl_scan_t scan_t;

bool scan_valid(double z);
void dataCallback(const nav_msgs::Odometry& msg, const sensor_msgs::LaserScan& scan);
bool update_node(sl_vector_t u_t[2]);
void update_motion(sl_vector_t u_t[2]);

Eigen::MatrixXd compute_points(sl_node_t& node_i);
double norm2(Eigen::Vector2d& p, Eigen::Vector2d& q);
void get_correspondences(Eigen::MatrixXd& cores, Eigen::MatrixXd& A, Eigen::MatrixXd& B);
void crossCovariance_Mean(Eigen::MatrixXd& cores, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::Vector2d& mean_A, Eigen::Vector2d& mean_B, Eigen::MatrixXd& H);
double error_ICP(Eigen::MatrixXd& cores, Eigen::MatrixXd& pcl_ref, Eigen::MatrixXd& pcl_cur, Eigen::Matrix2d& R, Eigen::Vector2d& t);
double error_ICP_plus(Eigen::MatrixXd& pcl_ref, Eigen::MatrixXd& pcl_cur, Eigen::Matrix2d& R, Eigen::Vector2d& t);
void vanilla_ICP(sl_node_t& node_i, sl_node_t& node_j);
void inv_covariance_ICP(Eigen::MatrixXd& pcl_ref, Eigen::MatrixXd& pcl_cur, Eigen::Matrix2d& R, Eigen::Vector2d& t, sl_matrix_t& inv_cov_matrix);

Eigen::Vector3d error_func(Eigen::Vector3d& x_i, Eigen::Vector3d& x_j, Eigen::Vector3d& z_ij);
Eigen::Matrix<double, 3, 6> jacobian_func(Eigen::Vector3d& x_i, Eigen::Vector3d& x_j, Eigen::Vector3d& z_ij);
double cost_func(vector<sl_edge_t>& edge_t_, Eigen::VectorXd& x);
void optimization(sl_graph_t& graph_t_);

void ray_tracing(sl_node_t& node_i, nav_msgs::OccupancyGrid& map_t, vector<double>& log_map_t);
void mapping(sl_graph_t& graph_t_, vector<double>& log_map_t, nav_msgs::OccupancyGrid& map_t, nav_msgs::Path& pose_graph_t);
void init_slam(vector<double>& log_map_t, nav_msgs::OccupancyGrid& map_t, nav_msgs::Path& pose_graph_t);

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