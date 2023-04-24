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

struct Pose{
    double x;
    double y;
    double theta;
};
struct LaserData{
    double angle_min;
    double angle_max;
    double angle_increment;
    double range_min;
    double range_max;
    Pose position;
    vector<double> ranges;
};
struct Particle{
    Pose pose;
    double weight;
};

/*Parameters*/
double anpha1, anpha2, anpha3, anpha4, anpha5;
double sigma_hit;
double z_hit, z_rand, z_max;
double max_beams;

double anpha_slow, anpha_fast;
double dist_max;
double min_trans;
double min_rot;
int min_particles;
int max_particles;
int throttle_scan;
int outlier_point;
double range_max;
double bin_size_x, bin_size_y, bin_size_theta;
double kld_eps, kld_delta;
double init_cov_x, init_cov_y, init_cov_theta;
double frequency_publish;
double init_pose_x, init_pose_y, init_pose_theta;

double err_min;
double match_rate;

#define occupied 100
#define free 0
#define unknown -1

#define omni 0
#define diff 1
#define empt 0
#define no_empt 1

int model_type;
bool lookup_table;
bool uniform_pdf;
string lookup_table_path;
bool save_last_pose;
bool set_init_pose;
string last_pose_path;

string base_frame;
string map_frame;

double laser_pose_x;
double laser_pose_y;
double laser_pose_theta;

bool inverse_laser;
bool agumented_mcl;
bool get_entropy;

nav_msgs::OccupancyGrid occ_map;
LaserData laser_scan;
nav_msgs::Odometry odom;

vector<Particle> set_particle_t;
vector<double> set_weight;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void dataCallback(const nav_msgs::Odometry& msg, const sensor_msgs::LaserScan& scan);

void get_lookuptable();
void uniform_sample(nav_msgs::OccupancyGrid& map);
void normal_sample();

inline double normalize(double z);
inline double angle_diff(double a, double b);
Pose motion_model(Pose u_t[2], Pose q_t_1);
double likelihoodfield_model(LaserData& scan_t, Pose q_t, nav_msgs::OccupancyGrid& map);

void AugmentedMCL(vector<Particle>& S_t_1, Pose u_t[2], LaserData& scan_t, nav_msgs::OccupancyGrid& map);
void KLDSamplingMCL(vector<Particle>& S_t_1, Pose u_t[2], LaserData& scan_t, nav_msgs::OccupancyGrid& map);

geometry_msgs::PoseWithCovarianceStamped MeanAndCovariance();
double computeEntropy();
sensor_msgs::PointCloud PointCloudExport(LaserData& scan_t, geometry_msgs::PoseWithCovarianceStamped q_t, nav_msgs::OccupancyGrid& map);

bool update_filter(Pose u_t[2]);
void update_motion(Pose u_t[2]);

bool _map = false, _data = false;
bool first_time = true, error_ = false;
bool init_localization = false;

bool map_valid(int idx, int idy);