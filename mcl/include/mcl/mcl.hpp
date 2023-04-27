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

pf_set_sample_t set_particle_t;
pf_scan_t set_scan_t;
vector<double> set_occ_dist;

#define occupied 100
#define free 0
#define unknown -1

#define omni 0
#define diff 1
#define empt 0
#define no_empt 1

double range_min;
double range_max;
double angle_increment;
double angle_min;
double angle_max;

double laser_pose_x;
double laser_pose_y;
double laser_pose_theta;

int model_type;
double anpha1, anpha2, anpha3, anpha4, anpha5;
double sigma_hit;
double z_hit, z_rand, z_max;
double max_beams;
double lamda_hit, lamda_short, lamda_max, lamda_rand;
double max_occ_dist;

double anpha_slow, anpha_fast;
double min_trans;
double min_rot;
int min_particles;
int max_particles;
int throttle_scan;

double bin_size_x, bin_size_y, bin_size_theta;
double kld_eps, kld_delta;
double init_cov_x, init_cov_y, init_cov_theta;
double frequency_publish;
double init_pose_x, init_pose_y, init_pose_theta;
double range_sample;

double converged_distance;
double beam_skip_threshold, beam_skip_distance, beam_skip_error_threshold;

double err_min;
double match_rate;

bool lookup_table;
bool uniform_pdf_submap;
bool uniform_pdf_allmap;
bool inverse_laser;
bool likelihoodfield;
bool agumented_mcl;
bool entropy;

bool save_last_pose;
bool set_init_pose;

string lookup_table_path;
string last_pose_path;
string base_frame;
string map_frame;

nav_msgs::OccupancyGrid occ_map;
sensor_msgs::LaserScan laser_scan;
nav_msgs::Odometry odom;

bool _map = false, _data = false;
bool first_time = true, error_ = false;
bool init_localization = false;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void dataCallback(const nav_msgs::Odometry& msg, const sensor_msgs::LaserScan& scan);
bool map_valid(int idx, int idy);
void get_lookuptable();

inline double normalize(double z);
inline double angle_diff(double a, double b);
void compute_mean_covariance(pf_set_sample_t& s);
void uniform_sample_allmap(nav_msgs::OccupancyGrid& map);
void uniform_sample_submap(nav_msgs::OccupancyGrid& map);
void normal_sample(nav_msgs::OccupancyGrid& map);
pf_vector_t motion_model(pf_vector_t u_t[2], pf_vector_t q_t_1);

void scan_handle(pf_set_sample_t& s, sensor_msgs::LaserScan& scan_t, nav_msgs::OccupancyGrid& map);
double ray_tracing(double x_, double y_, double anpha, nav_msgs::OccupancyGrid& map);
double likelihoodfield_model(pf_scan_t& scan_t, pf_vector_t& q_t, nav_msgs::OccupancyGrid& map);
double beams_model(pf_scan_t& scan_t, pf_vector_t& q_t, nav_msgs::OccupancyGrid& map);

void AugmentedMCL(pf_set_sample_t& S_t_1, pf_vector_t u_t[2], sensor_msgs::LaserScan& scan_t, nav_msgs::OccupancyGrid& map);
void KLDSamplingMCL(pf_set_sample_t& S_t_1, pf_vector_t u_t[2], sensor_msgs::LaserScan& scan_t, nav_msgs::OccupancyGrid& map);
double computeEntropy();
bool update_filter(pf_vector_t u_t[2]);
void update_motion(pf_vector_t u_t[2]);
void mcl_publisher(geometry_msgs::PoseArray& pose_arr, geometry_msgs::PoseWithCovarianceStamped& q_t, sensor_msgs::PointCloud& pcl, 
    pf_set_sample_t& s, pf_scan_t& scan_t, nav_msgs::OccupancyGrid& map);
