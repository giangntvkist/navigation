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

// Eigen::Matrix3d inverse_covariance_func(...) {
    
// }
Eigen::Vector3d error_func(Eigen::Vector3d& x_i, Eigen::Vector3d& x_j, Eigen::Vector3d& z_ij) {
    Eigen::Matrix2d R_i;
    Eigen::Matrix2d R_j;
    Eigen::Matrix2d R_ij;
    double theta_i = x_i(2);
    double theta_j = x_j(2);
    double theta_ij = z_ij(2);
    R_i << cos(theta_i), -sin(theta_i),
        sin(theta_i), cos(theta_i);

    R_j << cos(theta_j), -sin(theta_j),
        sin(theta_j), cos(theta_j);
    
    R_ij << cos(theta_ij), -sin(theta_ij),
        sin(theta_ij), cos(theta_ij);

    Eigen::Vector2d t_i;
    Eigen::Vector2d t_j;
    Eigen::Vector2d t_ij;
    t_i = x_i.head(2);
    t_j = x_j.head(2);
    t_ij = z_ij.head(2);

    Eigen::Vector3d e_ij;
    e_ij << R_ij.transpose()*(R_i.transpose()*(t_j - t_i) - t_ij),
        theta_j - theta_i - theta_ij;
    return e_ij;
}

Eigen::Matrix<double, 3, 6> jacobian_func(Eigen::Vector3d& x_i, Eigen::Vector3d& x_j, Eigen::Vector3d& z_ij) {
    Eigen::Matrix2d R_i, R_ij, diff_R_i;
    Eigen::Vector2d t_i, t_j;
    Eigen::Matrix3d A_ij, B_ij;
    A_ij.setZero(3, 3);
    B_ij.setZero(3, 3);
    double theta_i = x_i(2);
    double theta_ij = z_ij(2);
    R_i << cos(theta_i), -sin(theta_i),
        sin(theta_i), cos(theta_i);
    
    R_ij << cos(theta_ij), -sin(theta_ij),
        sin(theta_ij), cos(theta_ij);

    diff_R_i << -sin(theta_i), -cos(theta_i),
        cos(theta_i), -sin(theta_i);

    t_i = x_i.head(2);
    t_j = x_j.head(2);

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

double cost_func(sl_graph_t& graph_t_) {
    sl_node_t node_i, node_j;
    sl_edge_t edge_ij;

    Eigen::Vector3d x_i, x_j, z_ij;
    Eigen::Vector3d e_ij;
    Eigen::Matrix3d omega_ij;

    int num_edges = graph_t_.set_edge_t.size();
    double cost_value = 0;
    for(int k = 0; k < num_edges; k++) {
        edge_ij = graph_t_.set_edge_t[k];
        z_ij << edge_ij.z.v[0],
            edge_ij.z.v[1],
            edge_ij.z.v[2];

        // omega_ij = ...

        node_i = graph_t_.set_node_t[edge_ij.i];
        x_i << node_i.pose.v[0],
            node_i.pose.v[1],
            node_i.pose.v[2];

        node_j = graph_t_.set_node_t[edge_ij.j];
        x_j << node_j.pose.v[0],
            node_j.pose.v[1],
            node_j.pose.v[2];

        e_ij = error_func(x_i, x_j, z_ij);
        cost_value += e_ij.transpose()*omega_ij*e_ij;
    }
    return cost_value;
}

// void optimization(sl_graph_t& graph_t_) {
//     int num_nodes = ...;
//     Eigen::Matrix<double, Dynamic, 1> b(3*num_nodes), delta_x(3*num_nodes);
//     Eigen::SparseMatrix<double> H(3*num_nodes, 3*num_nodes);
//     Eigen::Matrix3d A_ij, B_ij, omega_ij;
//     Eigen::Matrix<double, 3, 6> J_ij;
//     Eigen::Vector3d x_i, x_j, z_ij, e_ij;
//     while(...||num_inter < max_inter) {
//         b.setZero();
//         H.setZero();
//         for(int k = 0; k < num_edges; k++) {
//             i = ...
//             j = ...
//             J_ij = jacobian_func(x_i, x_j, z_ij);
//             A_ij = J_ij.leftCols(3);
//             B_ij = J_ij.rightCols(3);


//             e_ij = error_func(x_i, x_j, z_ij);
//             b.segment(3*i,3) += A_ij.transpose()*omega_ij*e_ij;
//             b.segment(3*j,3) += B_ij.transpose()*omega_ij*e_ij;
//         }

//     }
// }
