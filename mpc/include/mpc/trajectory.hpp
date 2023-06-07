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

#include <random>
#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>

#include <casadi/casadi.hpp>

using namespace std;

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

struct Polygon{
    vector<Point> vertices;
    Point centroid;
    double radius;
};

double V_max;
double at_max;
double V_wheelmax;
double mass;
double W_max;
double F_max;
double L;
double wheel_radius;
double J_max;

vector<Point> points;
Point target_pose;
int num_point;
State lz_pose;

class Trajectory {
    public:
        int M;
        vector<Point> c0, c1, c2, c3, c4, c5;
        vector<double> U, T, arc_length;

        void path_planner(vector<Point>& p1, State& p2);
        double func(double u, int idx);
        double func_curve(double u, int idx);
        double inter_Romberg(double a, double b, int idx);
        void split_spline(double V_init, double V_end);

        State scurve_trajectory(double t);
        State bezier_trajectory(double t);
};
