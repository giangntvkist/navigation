#pragma once
#include "mpc/mpc.hpp"
class MPC {
    private:

    public:
        MPC() {

        };
        ~MPC() {};
};

void MPC::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    Point point;
    points.clear();
    for(int i = msg->poses.size()-1; i >= 0; i--) {
         point.x = msg->poses[i].pose.position.x;
         point.y = msg->poses[i].pose.position.y;
         points.push_back(point);
    }

    target_pose = points.back();
    num_point = points.size();
    path_ = true;
}

void MPC::x_poseCallback(const std_msgs::Float64& msg) {
    lz_pose.x = msg.data/100;
    lzpose_ = true;
}

void MPC::y_poseCallback(const std_msgs::Float64& msg) {
    lz_pose.y = msg.data/100;
}

void MPC::theta_poseCallback(const std_msgs::Float64& msg) {
    double z = msg.data*PI/180;
    lz_pose.theta = atan2(sin(z), cos(z));
}
