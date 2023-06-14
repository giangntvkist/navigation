#pragma once
#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

#include <costmap_converter/ObstacleArrayMsg.h>

#include <random>
#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>

#include <casadi/casadi.hpp>

using namespace std;
using namespace casadi;

struct Point{
    double x;
    double y;
    double z;
};

struct State{
    double x;
    double y;
    double theta;

    double vx;
    double vy;
    double w;

    double ax;
    double ay;
};

struct Vector{
    double nx;
    double ny;
};

struct Obstacle{
    double x;
    double y;
    double theta;
    double v;
    double radius;
    vector<Point> vertices;
};

#define bezier_ 1
#define scurve_ 2
#define INF 1e4
// #define cm_m 1.0
#define cm_m 100.0

vector<Point> global_path;
vector<Obstacle> set_obs, set_obs_nearest;
State lz_pose;
Point target_pose;
int num_waypoints;
double V_wheel[3];

ros::Publisher vel1_pub, vel2_pub, vel3_pub;
ros::Publisher tra_pub, path_predict_pub;

ros::Subscriber path_sub;
ros::Subscriber x_pose_sub, y_pose_sub, theta_pose_sub;
ros::Subscriber obs_sub, wheel_vel_sub;

// ros::Subscriber pose_sub;
// ros::Publisher vel_pub;

bool path_;
bool lzpose_, wheel_velocity_;
bool obstacles_, control_, trajectory_;

int M;
vector<Point> c0, c1, c2, c3, c4, c5;
vector<double> U, T, arc_length;

double V_max;
double at_max;
double V_wheelmax;
double mass;
double W_max;
double F_max;
double L;
double wheel_radius;
double J_max;

double k_curve;

string map_frame;
int trajectory_type;

double T_sample;
int N_predictsize;
int N_predictcontrol;
double e_safety;
int N_obstacles, N_maxobstacles;

double publish_frequency;
int max_iter;

double kernel_size, max_dist;
double err_goal;

double t;

void pathCallback(const nav_msgs::Path& msg);
void x_poseCallback(const std_msgs::Float64& msg);
void y_poseCallback(const std_msgs::Float64& msg);
void theta_poseCallback(const std_msgs::Float64& msg);
void poseCallback(const nav_msgs::Odometry& msg);
void velocityCallback(const std_msgs::Float64MultiArray& msg);

bool compare_obstacles(Obstacle p1, Obstacle p2);
void get_obstacles(Obstacle& p);

void quintic_bezier_splines(vector<Point>& path);
double func(double u, int idx);
double func_curve(double u, int idx);
double inter_Romberg(double (*f)(double, int), double a, double b, int idx);
void numerical_velocity_profile(double V_init, double V_end);
State bezier_trajectory(double t);
State scurve_trajectory(double t);

void trajectory_publisher();
void get_obstacles_nearest(vector<Obstacle>& set_obs_, vector<Obstacle>& set_obs_nearest_);


