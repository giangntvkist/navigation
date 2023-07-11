#pragma once
#include "vk_slam/vk_slam.hpp"

double normalize(double z) {
    return atan2(sin(z), cos(z));
}

double angle_diff(double a, double b) {
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

Eigen::Matrix<double, 3, 6> jacobian_func(sl_node_t& x_i, sl_node_t& x_j, sl_edge_t& z_ij) {
    double theta_i = x_i.pose.v[2];
    double theta_ij = z_ij.z.v[2];
    Eigen::Matrix2d R_i;
    Eigen::Matrix2d R_ij;
    Eigen::Matrix2d diff_R_i;

    Eigen::Vector2d t_i;
    Eigen::Vector2d t_j;

    Eigen::Matrix3d A_ij;
    Eigen::Matrix3d B_ij;

    R_i << cos(theta_i), -sin(theta_i),
        sin(theta_i), cos(theta_i);
    
    R_ij << cos(theta_ij), -sin(theta_ij),
        sin(theta_ij), cos(theta_ij);

    diff_R_i << -sin(theta_i), -cos(theta_i),
        cos(theta_i), -sin(theta_i);

    t_i << x_i.pose.v[0],
        x_i.pose.v[1];

    t_j << x_j.pose.v[0],
        x_j.pose.v[1];

    A_ij.setZero(3, 3);
    B_ij.setZero(3, 3);

    A_ij.block(0,0,2,2) = -R_ij.transpose()*R_i.transpose();
    A_ij.block(0,2,2,1) = R_ij.transpose()*diff_R_i.transpose()*(t_j - t_i);
    A_ij(2,2) = -1;

    B_ij.block(0,0,2,2) = R_ij.transpose()*R_i.transpose();
    B_ij(2,2) = 1;

    Eigen::Matrix<double, 3, 6> J_ij;
    J_ij.leftCols(3) = A_ij;
    J_ij.rightCols(3) = B_ij;
    return J_ij;
}
// Eigen::Matrix3d inverse_covariance_func(sl_matrix_t& m) {
    
// }
void optimization_func(sl_graph_t& graph_t_) {

}