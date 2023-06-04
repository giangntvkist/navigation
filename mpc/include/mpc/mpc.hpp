#pragma once
#include "mpc/trajectory.hpp"

struct Polygon{
    vector<Point> vertices;
    Point centroid;
    double radius;
};

class MPC {
    private:
        ros::Subscriber path_sub;
        ros::Subscriber x_pose_sub;
        ros::Subscriber y_pose_sub;
        ros::Subscriber theta_pose_sub;
        ros::Subscriber obs_sub;

        bool mode1, mode2;
        bool polygon_, control_;

        double T_sample;
        int N_predictsize;
        int N_predictcontrol;

        void pathCallback(const nav_msgs::Path::ConstPtr& msg);
        void x_poseCallback(const std_msgs::Float64& msg);
        void y_poseCallback(const std_msgs::Float64& msg);
        void theta_poseCallback(const std_msgs::Float64& msg);
    public:
        MPC() {

        };
        ~MPC() {};
};

