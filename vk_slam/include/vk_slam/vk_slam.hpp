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

struct sl_corr_t {
	int i;
	/** Closest point in the other scan. */
	int j1;
	/** Second closest point in the other scan. */
	int j2;
    /** Squared distance from p(i) to point j1 */
	double dist2_j1;
    double weight;
};

/* Define laser scan data: distance data & ray angle*/
struct sl_scan_t {
	vector<sl_vector_t> ranges;
};

struct sl_point_cloud_t {
    vector<sl_vector_t> pcl;
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
#define un_valid -1
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
bool first_time;
bool inverted_laser;
bool loop_closure_detected;
bool overlap_best;

int max_inter_ICP;
double match_rate_ICP;

int max_inter;
double converged_graph;
double map_update_interval;

double min_trans, min_rot;
int map_width, map_height;
double map_resolution;
double z_hit, sigma;
double min_cumulative_distance;

double delta_x, delta_y, delta_theta; /* Using for calculating approximate Hessian matrix */

ros::Publisher map_pub, pose_graph_pub;

nav_msgs::Odometry odom;
sl_scan_t scan_t;

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

// bool compare(const sl_corr_t& a, sl_corr_t& b){
//    return a.dist2_j1 < b.dist2_j1;
// }

bool scan_valid(double z);
void dataCallback(const nav_msgs::Odometry& msg, const sensor_msgs::LaserScan& scan);
bool update_node(sl_vector_t u_t[2]);
void update_motion(sl_vector_t u_t[2]);

Eigen::Vector3d error_func(Eigen::Vector3d& x_i, Eigen::Vector3d& x_j, Eigen::Vector3d& z_ij);
Eigen::Matrix<double, 3, 6> jacobian_func(Eigen::Vector3d& x_i, Eigen::Vector3d& x_j, Eigen::Vector3d& z_ij);
double cost_func(vector<sl_edge_t>& edge_t_, Eigen::VectorXd& x);
void optimization(sl_graph_t& graph_t_);
void cov_func(sl_graph_t& graph_t_);
void thread_func(sl_graph_t& graph_t_);

void compute_points(sl_node_t& node_i, sl_point_cloud_t& pcl_cur);
void transform_pcl(sl_point_cloud_t& pcl_cur, sl_point_cloud_t& pcl_cur_w, sl_vector_t& trans);
void get_correspondences(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur_w, vector<sl_corr_t>& cores);
// double gold_rate(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur_w, vector<sl_corr_t>& cores);
void compute_cov_mean_ICP(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur, vector<sl_corr_t>& cores,
    sl_vector_t& mean_ref, sl_vector_t& mean_cur, double (&H)[2][2]);
double compute_sum_error_ICP(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur, vector<sl_corr_t>& cores);
void vanilla_ICP(sl_node_t& node_i, sl_node_t& node_j, sl_edge_t& edge_ij);
double compute_sum_error_ICP_plus(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur, sl_vector_t& trans);
void inverse_cov_ICP(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur, sl_vector_t& trans, sl_matrix_t& inv_cov_matrix, int num_cores);

bool check_loop_closure(sl_node_t& node_i, sl_node_t& node_j);
double mahalanobis_distance(sl_node_t& node_i, sl_node_t& node_j);
void compute_loop_constraint(sl_node_t& node_i, sl_node_t& node_j, sl_edge_t& edge_ij);
void detect_loop_closure(sl_graph_t& graph_t_);

void ray_tracing(sl_node_t& node_i, nav_msgs::OccupancyGrid& map_t, vector<double>& log_map_t);
void mapping(sl_graph_t& graph_t_, vector<double>& log_map_t, nav_msgs::OccupancyGrid& map_t, nav_msgs::Path& pose_graph_t);
void init_slam(vector<double>& log_map_t, nav_msgs::OccupancyGrid& map_t, nav_msgs::Path& pose_graph_t);
void printf_matrix(sl_matrix_t& A);
