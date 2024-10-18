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

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

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
	int j;
	double dist2;
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
#define my_inf 1e4
#define un_valid -1
int model_type; /* Type of model robot: differential or omnidirectional robot */

double range_min, range_max, angle_increment, angle_min, angle_max;
double laser_pose_x, laser_pose_y, laser_pose_theta;
double init_pose_x, init_pose_y, init_pose_theta;

double min_trans, min_rot;
double map_width, map_height, map_resolution;
double map_update_interval;
int max_inter, kernel_size;
double dist_threshold;

double delta_x, delta_y, delta_theta; /* Using for calculating approximate Hessian matrix */

int throttle_scan;
string map_frame, base_frame, odom_frame;

bool sensor_data, map_data;
bool first_time;
bool inverted_laser;

bool loop_closure_detected;
bool overlap_best;
double match_rate, min_cumulative_distance;
bool optimized;

sl_scan_t scan_t;
sl_vector_t odom;

ros::Publisher map_pub, pose_graph_pub;
boost::mutex mu;

bool scan_valid(double z);
bool map_valid(int idx, int idy, nav_msgs::OccupancyGrid& map_t);
double normalize(double z);
double angle_diff(double a, double b);
void dataCallback(const nav_msgs::Odometry& msg, const sensor_msgs::LaserScan& scan);

void ray_tracing(sl_node_t& node_i, nav_msgs::OccupancyGrid& map_t, vector<double>& log_map_t);
void mapping(sl_graph_t& graph_t, vector<double>& log_map_t, nav_msgs::OccupancyGrid& map_t);
void pose_graph_visualization(sl_graph_t& graph_t, visualization_msgs::Marker& node, visualization_msgs::Marker& edge, visualization_msgs::MarkerArray& SetOfMarker);
void init_slam(vector<double>& log_map_t, nav_msgs::OccupancyGrid& map_t, visualization_msgs::Marker& node, visualization_msgs::Marker& edge, visualization_msgs::MarkerArray& SetOfMarker, int color);
bool update_node(sl_vector_t u_t[2]);
void update_motion(sl_vector_t u_t[2], sl_vector_t& q_t);
void printf_matrix(sl_matrix_t& A);

void compute_points(sl_node_t& node_i, sl_point_cloud_t& pcl_cur);
void transform_pcl(sl_point_cloud_t& pcl_cur, sl_point_cloud_t& pcl_cur_w, sl_vector_t& trans);
double norm2(sl_vector_t& p, sl_vector_t& q);
void correspondences(sl_point_cloud_t& pcl_cur_w, nav_msgs::OccupancyGrid& map_t, vector<sl_corr_t>& cores);
void compute_cov_mean_ICP(sl_point_cloud_t& pcl_cur, nav_msgs::OccupancyGrid& map_t, vector<sl_corr_t>& cores,
    sl_vector_t& mean_ref, sl_vector_t& mean_cur, double (&H)[2][2]);
double compute_sum_error_ICP(nav_msgs::OccupancyGrid& map_t, sl_point_cloud_t& pcl_cur_w, vector<sl_corr_t>& cores);
void vanilla_ICP(sl_node_t& node_i, sl_node_t& node_j, sl_edge_t& edge_ij, nav_msgs::OccupancyGrid& map_t);

double compute_sum_error_ICP_plus(nav_msgs::OccupancyGrid& map_t, sl_point_cloud_t& pcl_cur, sl_vector_t& trans);
void inverse_cov_ICP(nav_msgs::OccupancyGrid& map_t, sl_point_cloud_t& pcl_cur, sl_vector_t& trans, sl_matrix_t& inv_cov_matrix, int num_cores);

void corr_scan_to_scan(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur_w, vector<sl_corr_t>& cores);
double sum_error(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur_w, vector<sl_corr_t>& cores);
void covariance_mean(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur, vector<sl_corr_t>& cores,
    sl_vector_t& mean_ref, sl_vector_t& mean_cur, double (&H)[2][2]);
bool check_loop_closure(sl_node_t& node_i, sl_node_t& node_j, sl_edge_t& edge_ij);
double mahalanobis_distance(sl_node_t& node_i, sl_node_t& node_j);
void detect_loop_closure(sl_graph_t& graph_t_);
void thread_func(sl_graph_t& graph_t_, vector<double>& log_map_t, nav_msgs::OccupancyGrid& map_t);

Eigen::Vector3d error_func(Eigen::Vector3d& x_i, Eigen::Vector3d& x_j, Eigen::Vector3d& z_ij);
Eigen::Matrix<double, 3, 6> jacobian_func(Eigen::Vector3d& x_i, Eigen::Vector3d& x_j, Eigen::Vector3d& z_ij);
double cost_func(vector<sl_edge_t>& edge_t, Eigen::VectorXd& x);
void optimization(sl_graph_t& graph_t_);
void cov_func(sl_graph_t& graph_t_);

