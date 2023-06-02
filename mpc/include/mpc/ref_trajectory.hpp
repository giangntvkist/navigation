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

void pathCallback(const nav_msgs::Path::ConstPtr& msg);
void x_poseCallback(const std_msgs::Float64& msg);
void y_poseCallback(const std_msgs::Float64& msg);
void theta_poseCallback(const std_msgs::Float64& msg);

vector<Point> points;
Point target_pose;
int num_point;
bool path_;
bool lzpose_;
ros::Subscriber path_sub;

class Trajectory {
    private:
        double V_limit, a_limit, J_limit;
        State scurve_trajectory(double t);
        State bezier_trajectory(double t);
    public:
        Trajectory () {

        };
        ~Trajectory() {};
};

State Trajectory::scurve_trajectory(double t) {
    double kx, ky;
    double d, v, a, J;
    double t0, t1, t2, t3, t4, t5, t6, t7, T;
    State q_t;
    for(int i = 0; i < num_point-1; i++) {
        kx = points[i+1].x - points[i].x;
        ky = points[i+1].y - points[i].y;
        d = sqrt(pow(kx, 2) + pow(ky, 2));

        v = V_limit/d;
        a = a_limit/d;
        J = J_limit/d;
        if(v*v/a + a*v/J >= 1 || a/J >= v/a) {
            ROS_WARN("Choosing the wrong parameter v a J!");
        }
        T = a/J + v/a + 1/v;
        t0 = 0;
        t1 = a/J;
        t2 = v/a;
        t3 = v/a + a/J;
        t4 = T - t3;
        t5 = T - t2;
        t6 = T - t1;
        t7 = T;
        double n, m, s;
        if(t <= t1) {
            n = J*t;
            m = J*t*t/2;
            s = J*pow(t, 3)/6
        }else if(t > t1 && t <= t2) {
            n = a;
            m = a*t - a*a/(2*J);
            s = (a*t*t)/2 + pow(a, 3)/(6*J*J) - (a*a*t)/(2*J);
        }else if(t > t2 && t <= t3) {
            n = a - J*(t - v/a);
            m = v - a*a/(2*J) - ((v - a*t)*(2*a*a - a*J*t + J*v))/(2*a*a);
            s = (-pow(J*a*t, 3) + 3*pow(J, 3)*pow(a*t, 2)*v - 3*pow(J, 3)*a*t*pow(v, 2) + pow(J*v, 3) + 3*J*J*pow(a*a*t, 2) - 3*J*pow(a, 5)*t + pow(a, 6))/(6*J*J*pow(a, 3));
        }else if(t > t3 && t <= t4) {
            n = 0;
            m = v;
            s = -(v*(a*a - 2*J*a*t + J*v))/(2*J*a);
        }else if(t > t4 && t <= t5) {
            n = -J*(t - T + a/J + v/a);
            m = v - pow((J*v + a*a - J*T*a + J*a*t), 2)/(2*J*a*a);
            s = -pow(J*T*a, 3) + 3*pow(J*a, 3)*T*T*t + 3*pow(J, 3)*pow(a*T, 2)*v - 3*pow(J*a, 3)*T*t*t - 6*pow(J, 3)*T*a*a*t*v - 3*pow(J, 3)*T*a*v*v + pow(J*a*t, 3) + 3*pow(J, 3)*pow(a*t, 2)*v;
            s += 3*pow(J, 3)*a*t*v*v + pow(J*v, 3) + 3*pow(J*T*a*a, 2) - 6*pow(J*a*a, 2)*T*t - 6*J*J*T*pow(a, 3)*v + 3*pow(J*a*a*t, 2) + 6*pow(J*a*v, 2) - 3*J*T*pow(a, 5);
            s += 3*J*pow(a, 5)*t + 6*J*pow(a, 4)*v + pow(a, 6);
            s = -s/(6*J*J*pow(a, 3));
        }else if(t > t5 && t <= t6) {
            n = -a;
            m = -(a*(a + 2*J*t - 2*J*T))/(2*J);
            s = ((v - T*a + a*t)*(J*v - a*a +J*T*a - J*a*t))/(2*J*a) - (-6*T*J*J*a*v + 9*J*J*v*v + 3*J*a*a*v + pow(a, 4))/(6*J*J*a);
        }else if(t > t6 && t <= t7) {
            n = J*(t - T + a/J) - a;
            m = (J*pow((T - t), 2))/2;
            s = pow(a, 3)/(6*J*J) - J*(pow(T, 3)/6 - (T*T*t)/2 + (T*t*t)/2 - pow(t, 3)/6) - (-6*T*J*J*a*v + 6*pow(J*v, 2) + 6*J*a*a*v + pow(a, 4))/(6*J*J*a);
        }else {
            t = T;
            n = J*(t - T + a/J) - a;
            m = (J*pow((T - t), 2))/2;
            s = pow(a, 3)/(6*J*J) - J*(pow(T, 3)/6 - (T*T*t)/2 + (T*t*t)/2 - pow(t, 3)/6) - (-6*T*J*J*a*v + 6*pow(J*v, 2) + 6*J*a*a*v + pow(a, 4))/(6*J*J*a);
        }
        q_t.ax = kx * n; q_t.ay = ky * n;
        q_t.vx = kx * m; q_t.vy = ky * m;
        q_t.x = points[i].x + kx * s; q_t.y = points[i].y + ky * s;
    }
    return q_t;
}