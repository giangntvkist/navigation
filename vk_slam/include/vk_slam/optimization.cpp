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

double cost_func(vector<sl_edge_t>& set_edge_t, Eigen::VectorXd& x) {
    sl_edge_t edge_ij;
    Eigen::Vector3d x_i, x_j, z_ij;
    Eigen::Vector3d e_ij;
    Eigen::Matrix3d omega_ij;

    int num_edges = set_edge_t.size();
    double cost_value = 0;
    for(int k = 0; k < num_edges; k++) {
        edge_ij = set_edge_t[k];
        z_ij << edge_ij.z.v[0],
            edge_ij.z.v[1],
            edge_ij.z.v[2];

        // omega_ij = ...

        x_i = x.segment(3*edge_ij.i, 3);
        x_j = x.segment(3*edge_ij.j, 3);
        e_ij = error_func(x_i, x_j, z_ij);
        cost_value += e_ij.transpose()*omega_ij*e_ij;
    }
    return cost_value;
}

void optimization(sl_graph_t& graph_t_) {
    int num_nodes = graph_t_.set_node_t.size();
    int num_edges = graph_t_.set_edge_t.size();

    Eigen::VectorXd b(3*num_nodes), x(3*num_nodes), delta_x(3*num_nodes);
    Eigen::SparseMatrix<double> H(3*num_nodes, 3*num_nodes);

    Eigen::Matrix3d A_ij, B_ij, omega_ij;
    Eigen::Matrix<double, 3, 6> J_ij;
    Eigen::Vector3d x_i, x_j, z_ij, e_ij;
    sl_node_t node_i, node_j;
    sl_edge_t edge_ij;

    Eigen::Matrix3d tmp_ij;
    Eigen::Matrix3d I;
    I.setIdentity();
    /* Initial guess x[] */
    for(int k = 0; k < 3*num_nodes; k++) {
        int i = k/3; // node i_th
        int j = k%3; // j = 0 -> x, j = 1 -> y, j = 2 -> theta
        x(k) = graph_t_.set_node_t[i].pose.v[j];
    }
    double cvl_k, cvl_k_1;
    double eps = inf;
    cvl_k = inf;
    int num_inter = 0;
    while(fabs(eps) > e_converged && num_inter < max_inter) {
        b.setZero();
        H.setZero();
        for(int k = 0; k < num_nodes; k++) {
            x(3*k+2) = normalize(x(3*k+2));
        }
        cvl_k = 0;
        for(int k = 0; k < num_edges; k++) {
            /* Edge ij from node i to node j*/
            edge_ij = graph_t_.set_edge_t[k];
            z_ij << edge_ij.z.v[0],
                edge_ij.z.v[1],
                edge_ij.z.v[2];
            int i = edge_ij.i;
            int j = edge_ij.j;

            /* Test omega */
            omega_ij = 1000*omega_ij.setIdentity();

            x_i = x.segment(3*edge_ij.i, 3);
            x_j = x.segment(3*edge_ij.j, 3);

            J_ij = jacobian_func(x_i, x_j, z_ij);
            A_ij = J_ij.leftCols(3);
            B_ij = J_ij.rightCols(3);

            /* Compute H[ii] += A_ij^T*omega_ij*A_ij */
            tmp_ij = A_ij.transpose()*omega_ij*A_ij;
            for(int n = 3*i; n < 3*i+3; n++) {
                for(int m = 3*i; m < 3*i+3; m++) {
                    H.coeffRef(n, m) += tmp_ij(n%3, m%3);
                }
            }

            /* Compute H[ij] += A_ij^T*omega_ij*B_ij */
            tmp_ij = A_ij.transpose()*omega_ij*B_ij;
            for(int n = 3*i; n < 3*i+3; n++) {
                for(int m = 3*j; m < 3*j+3; m++) {
                    H.coeffRef(n, m) += tmp_ij(n%3, m%3);
                }
            }
            
            /* Compute H[ji] += B_ij^T*omega_ij*A_ij */
            tmp_ij = B_ij.transpose()*omega_ij*A_ij;
            for(int n = 3*j; n < 3*j+3; n++) {
                for(int m = 3*i; m < 3*i+3; m++) {
                    H.coeffRef(n, m) += tmp_ij(n%3, m%3);
                }
            }
            
            /* Compute H[jj] += B_ij^T*omega_ij*B_ij */
            tmp_ij = B_ij.transpose()*omega_ij*B_ij;
            for(int n = 3*j; n < 3*j+3; n++) {
                for(int m = 3*j; m < 3*j+3; m++) {
                    H.coeffRef(n, m) += tmp_ij(n%3, m%3);
                }
            }

            e_ij = error_func(x_i, x_j, z_ij);
            /* Compute b[i] += A_ij^T*omega_ij*e_ij */
            b.segment(3*i,3) += A_ij.transpose()*omega_ij*e_ij;
            /* Compute b[j] += B_ij^T*omega_ij*e_ij */
            b.segment(3*j,3) += B_ij.transpose()*omega_ij*e_ij;

            /* Cost value */
            cvl_k += e_ij.transpose()*omega_ij*e_ij;
        }
        eps = cvl_k - cvl_k_1;
        cvl_k_1 = cvl_k;
        
        /* Compute H[11] += I */
        for(int n = 0; n < 3; n++) {
            for(int m = 0; m < 3; m++) {
                H.coeffRef(n, m) += I(n, m);
            }
        }
        H.makeCompressed();
        /* Solving */
        Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> chol(H); // Performs a Cholesky factorization of matrix H
        if (chol.info() != Success) {
            ROS_ERROR("Decomposition failed!");
            break;
        }else {
            delta_x = chol.solve(-b);
            if (chol.info() != Success) {
                ROS_ERROR("Solver failed!");
                break;
            }else {
                x += delta_x;
                num_inter += 1;
            }
        }
    }
    /* H* = H; H*[11] -= I */
    for(int k = 0; k < 3*num_nodes; k++) {
        int i = k/3;
        int j = k%3; 
        graph_t.set_node_t[i].pose.v[j] = x(k);
    }
}

void pose_graph_test(sl_graph_t& graph_t_) {
    sl_node_t p;
    p.pose.v[0] = -3.0;
    p.pose.v[1] = -2.0;
    p.pose.v[2] = M_PI/2;
    graph_t_.set_node_t.push_back(p);

    p.pose.v[0] = -3.2;
    p.pose.v[1] = 0;
    p.pose.v[2] = 1.75;
    graph_t_.set_node_t.push_back(p);

    p.pose.v[0] = -2.7;
    p.pose.v[1] = 1.9;
    p.pose.v[2] = 0.14;
    graph_t_.set_node_t.push_back(p);

    p.pose.v[0] = 2.8;
    p.pose.v[1] = 2.3;
    p.pose.v[2] = -1.69;
    graph_t_.set_node_t.push_back(p);

    p.pose.v[0] = 2.5;
    p.pose.v[1] = -2.1;
    p.pose.v[2] = -3.05;
    graph_t_.set_node_t.push_back(p);

    // p.pose.v[0] = -2.7;
    // p.pose.v[1] = -2.3;
    // p.pose.v[2] = 1.4;
    p.pose.v[0] = -2.2;
    p.pose.v[1] = 0.2;
    p.pose.v[2] = 1.75;
    graph_t_.set_node_t.push_back(p);

    sl_edge_t d;
    d.z.v[0] = 2.0;
    d.z.v[1] = 0.2;
    d.z.v[2] = 0.1792;
    d.i = 0;
    d.j = 1;
    graph_t_.set_edge_t.push_back(d);

    d.z.v[0] = 1.7805;
    d.z.v[1] = -0.8307;
    d.z.v[2] = -1.6100;
    d.i = 1;
    d.j = 2;
    graph_t_.set_edge_t.push_back(d);

    d.z.v[0] = 5.502;
    d.z.v[1] = -0.3714;
    d.z.v[2] = -1.8300;
    d.i = 2;
    d.j = 3;
    graph_t_.set_edge_t.push_back(d);

    d.z.v[0] = 4.4045;
    d.z.v[1] = 0.2254;
    d.z.v[2] = -1.3600;
    d.i = 3;
    d.j = 4;
    graph_t_.set_edge_t.push_back(d);

    // d.z.v[0] = 5.1965;
    // d.z.v[1] = -0.2765;
    // d.z.v[2] = 4.4500;
    d.z.v[0] = 4.4699;
    d.z.v[1] = -2.7202;
    d.z.v[2] = 4.800;
    d.i = 4;
    d.j = 5;
    graph_t_.set_edge_t.push_back(d);

    d.z.v[0] = 0;
    d.z.v[1] = 0;
    d.z.v[2] = 0;
    d.i = 5;
    d.j = 1;
    graph_t_.set_edge_t.push_back(d);
}
