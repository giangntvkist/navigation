#pragma once
#include <ros/ros.h>
#include <eigen3/Eigen/Core>

namespace slam {

inline double normalize(double z) {
    return atan2(sin(z), cos(z));
}

inline double anglediff(double a, double b) {
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

inline void UpdateMotion(Eigen::Matrix<double, 6, 1>& u_t_1,
                  Eigen::Matrix<double, 6, 1>& u_t,
                  Eigen::Matrix<double, 6, 1>& x,
                  std::string model_type) {
    if(model_type == "omni") {
        double delta_trans, delta_rot, delta_bearing;
        delta_trans = (u_t.head(3) - u_t_1.head(3)).norm();
        delta_rot = anglediff(u_t[5], u_t_1[5]);
        delta_bearing = anglediff(atan2(u_t[1] - u_t_1[1], u_t[0] - u_t_1[0]), u_t[5]) + x[5];

        Eigen::Matrix<double, 6, 1> delta_x = {delta_trans*cos(delta_bearing), delta_trans*sin(delta_bearing), 0.0, 0.0, 0.0, delta_rot};
        x += delta_x;
        x[5] = normalize(x[5]);
    } else if (model_type == "diff") {
        double delta_trans, delta_rot1, delta_rot2;
        delta_trans = (u_t.head(3) - u_t_1.head(3)).norm();
        if( delta_trans < 0.01) {
            delta_rot1 = 0.0;
        }else {
            delta_rot1 = anglediff(atan2(u_t[1] - u_t_1[1], u_t[0] - u_t_1[0]), u_t[5]);
        }
        delta_rot2 = anglediff(anglediff(u_t[5], u_t_1[5]), delta_rot1);
        Eigen::Matrix<double, 6, 1> delta_x = {delta_trans*cos(x[5] + delta_rot1), delta_trans*sin(x[5] + delta_rot1), 0.0, 0.0, 0.0, delta_rot1 + delta_rot2};
        x += delta_x;
        x[5] = normalize(x[5]);
    } else {
        ROS_ERROR("\"model_type\" parameter is unvalid, \"omni\" or \"diff\" !");
    }               
}

inline bool AddNode(Eigen::Matrix<double, 6, 1>& u_t_1,
             Eigen::Matrix<double, 6, 1>& u_t,
             double min_trans,
             double min_rot) {
    double trans = (u_t.head(3) - u_t_1.head(3)).norm();
    return (trans > min_trans || fabs(anglediff(u_t[5], u_t_1[5])) > min_rot);
}

}