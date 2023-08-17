#pragma once
#include "vk_slam/vk_slam.hpp"

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
        angle_diff(theta_j, theta_i) - theta_ij;
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

double cost_func(vector<sl_edge_t>& edge_t, Eigen::VectorXd& x) {
    sl_edge_t edge_ij;
    Eigen::Vector3d x_i, x_j, z_ij;
    Eigen::Vector3d e_ij;
    Eigen::Matrix3d omega_ij;

    int num_edges = edge_t.size();
    double cost_value = 0;
    for(int k = 0; k < num_edges; k++) {
        edge_ij = edge_t[k];
        z_ij << edge_ij.z.v[0],
            edge_ij.z.v[1],
            edge_ij.z.v[2];

        omega_ij << edge_ij.inv_cov.m[0][0], edge_ij.inv_cov.m[0][1], edge_ij.inv_cov.m[0][2],
            edge_ij.inv_cov.m[1][0], edge_ij.inv_cov.m[1][1], edge_ij.inv_cov.m[1][2],
            edge_ij.inv_cov.m[2][0], edge_ij.inv_cov.m[2][1], edge_ij.inv_cov.m[2][2];

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

    bool failed;
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
    while(fabs(eps) > converged_graph && num_inter < max_inter) {
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

            omega_ij << edge_ij.inv_cov.m[0][0], edge_ij.inv_cov.m[0][1], edge_ij.inv_cov.m[0][2],
                edge_ij.inv_cov.m[1][0], edge_ij.inv_cov.m[1][1], edge_ij.inv_cov.m[1][2],
                edge_ij.inv_cov.m[2][0], edge_ij.inv_cov.m[2][1], edge_ij.inv_cov.m[2][2];

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
            failed = true;
            break;
        }else {
            delta_x = chol.solve(-b);
            if (chol.info() != Success) {
                ROS_ERROR("Solver failed!");
                failed = true;
                break;
            }else {
                x += delta_x;
                num_inter += 1;
            }
        }
    }
    /* H* = H; H*[11] -= I */
    for(int n = 0; n < 3; n++) {
        for(int m = 0; m < 3; m++) {
            H.coeffRef(n, m) -= I(n, m);
        }
    }
    if(!failed) {
        for(int k = 0; k < 3*num_nodes; k++) {
            int i = k/3;
            int j = k%3; 
            graph_t_.set_node_t[i].pose.v[j] = x(k);
        }
    }
    // cout << " x " << x << endl;
}

void cov_func(sl_graph_t& graph_t_) {
    int num_nodes = graph_t_.set_node_t.size();
    int num_edges = graph_t_.set_edge_t.size();
    Eigen::VectorXd x(3*num_nodes);
    Eigen::SparseMatrix<double> H(3*num_nodes, 3*num_nodes);
    Eigen::SparseMatrix<double> H_red(3*(num_nodes -1), 3*(num_nodes - 1));

    Eigen::Matrix3d A_ij, B_ij, omega_ij;
    Eigen::Matrix<double, 3, 6> J_ij;
    Eigen::Vector3d x_i, x_j, z_ij;
    sl_node_t node_i, node_j;
    sl_edge_t edge_ij;
    Eigen::Matrix3d tmp_ij;
    Eigen::MatrixXd I(3*(num_nodes -1), 3*(num_nodes - 1));
    I.setIdentity();
    /* Initial guess x[] */
    for(int k = 0; k < 3*num_nodes; k++) {
        int i = k/3; // node i_th
        int j = k%3; // j = 0 -> x, j = 1 -> y, j = 2 -> theta
        x(k) = graph_t_.set_node_t[i].pose.v[j];
    }
    H.setZero();
    H_red.setZero();
    for(int k = 0; k < num_nodes; k++) {
        x(3*k+2) = normalize(x(3*k+2));
    }
    for(int k = 0; k < num_edges; k++) {
        /* Edge ij from node i to node j*/
        edge_ij = graph_t_.set_edge_t[k];
        z_ij << edge_ij.z.v[0],
            edge_ij.z.v[1],
            edge_ij.z.v[2];
        int i = edge_ij.i;
        int j = edge_ij.j;

        omega_ij << edge_ij.inv_cov.m[0][0], edge_ij.inv_cov.m[0][1], edge_ij.inv_cov.m[0][2],
            edge_ij.inv_cov.m[1][0], edge_ij.inv_cov.m[1][1], edge_ij.inv_cov.m[1][2],
            edge_ij.inv_cov.m[2][0], edge_ij.inv_cov.m[2][1], edge_ij.inv_cov.m[2][2];
        
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
    }
    H.makeCompressed();
    for(int i = 0; i < 3*(num_nodes - 1); i++) {
        for(int j = 0; j < 3*(num_nodes - 1); j++) {
            H_red.coeffRef(i, j) = H.coeffRef(i, j);
        }
    }
    H_red.makeCompressed();
    Eigen::MatrixXd H_red_inv;
    Eigen::Matrix3d H_ii;
    sl_matrix_t inv_cov_ii;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> chol(H_red);
    if(chol.info()!= Success) {
        ROS_INFO("H reduce inverse fail!");
    }else {
        H_red_inv = chol.solve(I);
        for(int i = 0; i < num_nodes - 1; i++) {
            H_ii = (H_red_inv.block(3*i, 3*i, 3, 3)).inverse();
            for(int j = 0; j < 3; j++) {
                for(int k = 0; k < 3; k++) {
                    inv_cov_ii.m[j][k] = H_ii(j, k);
                }
            }
            graph_t_.set_node_t[i].inv_cov = inv_cov_ii;
        }
    }
}

/*----------------------------------------------------------------------------------------------------*/
// void error_func(sl_vector_t& x_i, sl_vector_t& x_j, sl_vector_t& z_ij, sl_vector_t& e_ij) {
//     /** e_ij = R_ij^T *(R_i^T*(t_j - t_i) - t_ij) **/
//     double a, b;
//     a = cos(x_i.v[2])*(x_j.v[0] - x_i.v[0]) + sin(x_i.v[2])*(x_j.v[1] - x_i.v[1]);
//     b = -sin(x_i.v[2])*(x_j.v[0] - x_i.v[0]) + cos(x_i.v[2])*(x_j.v[1] - x_i.v[1]);

//     e_ij.v[0] = cos(z_ij.v[2])*(a - z_ij.v[0]) + sin(z_ij.v[2])*(b - z_ij.v[1]);
//     e_ij.v[1] = -sin(z_ij.v[2])*(a - z_ij.v[0]) + cos(z_ij.v[2])*(b - z_ij.v[1]);
//     e_ij.v[2] = angle_diff(x_j.v[2], x_i.v[2]) - z_ij.v[2];
// }

// void jacobian_func(sl_vector_t& x_i, sl_vector_t& x_j, sl_vector_t& z_ij, sl_matrix_t& A_ij, sl_matrix_t& B_ij) {
//     /** A_ij(0:1, 0:1) = -R_ij^T * R_i^T
//         A_ij(0:1, 2) = R_ij^T * diff_R_i^T * (t_j - t_i) 
//         A_ij(2, 2) = -1 **/
//     A_ij.m[0][0] = -(cos(z_ij.v[2])*cos(x_i.v[2]) - sin(z_ij.v[2])*sin(x_i.v[2]));
//     A_ij.m[0][1] = -(cos(z_ij.v[2])*sin(x_i.v[2]) + sin(z_ij.v[2])*cos(x_i.v[2]));

//     A_ij.m[1][0] = -(-sin(z_ij.v[2])*cos(x_i.v[2]) - cos(z_ij.v[2])*sin(x_i.v[2]));
//     A_ij.m[1][1] = -(-sin(z_ij.v[2])*sin(x_i.v[2]) + cos(z_ij.v[2])*cos(x_i.v[2]));

//     double a, b, c, d;
//     a = -cos(z_ij.v[2])*sin(x_i.v[2]) - sin(z_ij.v[2])*cos(x_i.v[2]);
//     b = cos(z_ij.v[2])*cos(x_i.v[2]) - sin(z_ij.v[2])*sin(x_i.v[2]);

//     c = sin(z_ij.v[2])*sin(x_i.v[2]) - cos(z_ij.v[2])*cos(x_i.v[2]);
//     d = -sin(z_ij.v[2])*cos(x_i.v[2]) - cos(z_ij.v[2])*sin(x_i.v[2]);

//     A_ij.m[0][2] = a*(x_j.v[0] - x_i.v[0]) + b*(x_j.v[1] - x_i.v[1]);
//     A_ij.m[1][2] = c*(x_j.v[0] - x_i.v[0]) + d*(x_j.v[1] - x_i.v[1]);

//     A_ij.m[2][0] = 0; A_ij.m[2][1] = 0; A_ij.m[2][2] = -1;

//     /** B_ij(0:1, 0:1) = R_ij^T*R_i^T 
//         B_ij(2,2) = 1 **/
//     B_ij.m[0][0] = -A_ij.m[0][0]; B_ij.m[0][1] = -A_ij.m[0][1]; B_ij.m[0][2] = 0;
//     B_ij.m[1][0] = -A_ij.m[1][0]; B_ij.m[1][1] = -A_ij.m[1][1]; B_ij.m[1][2] = 0;
//     B_ij.m[2][0] = 0; B_ij.m[2][1] = 0; B_ij.m[2][2] = 1;
// }

// /** CrossMatrix A*B */
// sl_matrix_t cross_matrix(sl_matrix_t& A, sl_matrix_t& B) {
//     sl_matrix_t C;
//     for(int i = 0; i < 3; i++) {
//         for(int j = 0; j < 3; j++) {
//             C.m[i][j] = 0;
//             for(int k = 0; k < 3; k++) {
//                 C.m[i][j] += A.m[i][k]*B.m[k][j];
//             }
//         }
//     }
//     return C;
// }

// /** Transpose Matrix A^T */
// sl_matrix_t transpose_matrix(sl_matrix_t& A) {
//     sl_matrix_t B;
//     for(int i = 0; i < 3; i++) {
//         for(int j = 0; j < 3; j++) {
//             if(i == j) {
//                 B.m[i][j] = A.m[i][j];
//             }else {
//                 B.m[i][j] = A.m[j][i];
//             }
//         }
//     }
//     return B;
// }

// /** Compute A^T*B*A */
// sl_matrix_t trip_cross_matrix(sl_matrix_t& A, sl_matrix_t& B) {
//     sl_matrix_t tmp;
//     sl_matrix_t matrix_trans;
//     matrix_trans = transpose_matrix(A);
//     tmp = cross_matrix(matrix_trans, B);
//     return cross_matrix(tmp, A);
// }

// /** Compute A^T*B*C */
// sl_matrix_t trip_cross_matrix(sl_matrix_t& A, sl_matrix_t& B, sl_matrix_t& C) {
//     sl_matrix_t tmp;
//     sl_matrix_t matrix_trans;
//     matrix_trans = transpose_matrix(A);
//     tmp = cross_matrix(matrix_trans, B);
//     return cross_matrix(tmp, C);
// }

// void optimization(sl_graph_t& graph_t_) {
//     int num_nodes = graph_t_.set_node_t.size();
//     int num_edges = graph_t_.set_edge_t.size();

//     Eigen::VectorXd b(3*num_nodes), x(3*num_nodes), delta_x(3*num_nodes);
//     Eigen::SparseMatrix<double> H(3*num_nodes, 3*num_nodes);

//     sl_matrix_t A_ij, B_ij, omega_ij;
//     sl_vector_t x_i, x_j, z_ij, e_ij;
    
//     sl_node_t node_i, node_j;
//     sl_edge_t edge_ij;

//     bool failed;
//     sl_matrix_t tmp_ij, matrix_trans;
//     sl_matrix_t I;
//     for(int i = 0; i < 3; i++) {
//         for(int j = 0; j < 3; j++) {
//             if(i != j) {
//                 I.m[i][j] = 0;
//             }else {
//                 I.m[i][j] = 1;
//             }
//         }
//     }

//     /* Initial guess x[] */
//     for(int k = 0; k < 3*num_nodes; k++) {
//         int i = k/3; // node i_th
//         int j = k%3; // j = 0 -> x, j = 1 -> y, j = 2 -> theta
//         x(k) = graph_t_.set_node_t[i].pose.v[j];
//     }

//     double cvl_k, cvl_k_1;
//     double eps = inf;
//     cvl_k = inf;
//     int num_inter = 0;
//     while(fabs(eps) > converged_graph && num_inter < max_inter) {
//         b.setZero();
//         H.setZero();
//         for(int k = 0; k < num_nodes; k++) {
//             x(3*k+2) = normalize(x(3*k+2));
//         }
//         cvl_k = 0;
//         for(int k = 0; k < num_edges; k++) {
//             /* Edge ij from node i to node j*/
//             edge_ij = graph_t_.set_edge_t[k];
//             z_ij = edge_ij.z;

//             int i = edge_ij.i;
//             int j = edge_ij.j;

//             omega_ij = edge_ij.inv_cov;

//             x_i.v[0] = x(3*i); x_i.v[1] = x(3*i+1); x_i.v[2] = x(3*i+2);
//             x_j.v[0] = x(3*j); x_j.v[1] = x(3*j+1); x_j.v[2] = x(3*j+2);

//             jacobian_func(x_i, x_j, z_ij, A_ij, B_ij);
//             /* Compute H[ii] += A_ij^T*omega_ij*A_ij */
//             tmp_ij = trip_cross_matrix(A_ij, omega_ij);
//             for(int n = 3*i; n < 3*i+3; n++) {
//                 for(int m = 3*i; m < 3*i+3; m++) {
//                     H.coeffRef(n, m) += tmp_ij.m[n%3][m%3];
//                 }
//             }

//             /* Compute H[ij] += A_ij^T*omega_ij*B_ij */
//             tmp_ij = trip_cross_matrix(A_ij, omega_ij, B_ij);
//             for(int n = 3*i; n < 3*i+3; n++) {
//                 for(int m = 3*j; m < 3*j+3; m++) {
//                     H.coeffRef(n, m) += tmp_ij.m[n%3][m%3];
//                 }
//             }

//             /* Compute H[ji] += B_ij^T*omega_ij*A_ij */
//             tmp_ij = trip_cross_matrix(B_ij, omega_ij, A_ij);
//             for(int n = 3*j; n < 3*j+3; n++) {
//                 for(int m = 3*i; m < 3*i+3; m++) {
//                     H.coeffRef(n, m) += tmp_ij.m[n%3][m%3];
//                 }
//             }

//             /* Compute H[jj] += B_ij^T*omega_ij*B_ij */
//             tmp_ij = trip_cross_matrix(B_ij, omega_ij);
//             for(int n = 3*j; n < 3*j+3; n++) {
//                 for(int m = 3*j; m < 3*j+3; m++) {
//                     H.coeffRef(n, m) += tmp_ij.m[n%3][m%3];
//                 }
//             }

//             error_func(x_i, x_j, z_ij, e_ij);
//             /* Compute b[i] += A_ij^T*omega_ij*e_ij */
//             matrix_trans = transpose_matrix(A_ij);
//             tmp_ij = cross_matrix(matrix_trans, omega_ij);
//             for(int n = 0; n < 3; n++) {
//                 b(3*i+n) = 0;
//                 for(int m = 0; m < 3; m++) {
//                     b(3*i+n) += tmp_ij.m[n][m]*e_ij.v[m];
//                 }
//             }
//             /* Compute b[j] += B_ij^T*omega_ij*e_ij */
//             matrix_trans = transpose_matrix(B_ij);
//             tmp_ij = cross_matrix(matrix_trans, omega_ij);
//             for(int n = 0; n < 3; n++) {
//                 b(3*j+n) = 0;
//                 for(int m = 0; m < 3; m++) {
//                     b(3*j+n) += tmp_ij.m[n][m]*e_ij.v[m];
//                 }
//             }

//             /* Cost value */
//             cvl_k += e_ij.v[0]*(e_ij.v[0]*omega_ij.m[0][0] + e_ij.v[1]*omega_ij.m[1][0] + e_ij.v[2]*omega_ij.m[2][0]);
//             cvl_k += e_ij.v[1]*(e_ij.v[0]*omega_ij.m[0][1] + e_ij.v[1]*omega_ij.m[1][1] + e_ij.v[2]*omega_ij.m[2][1]);
//             cvl_k += e_ij.v[2]*(e_ij.v[0]*omega_ij.m[0][2] + e_ij.v[1]*omega_ij.m[1][2] + e_ij.v[2]*omega_ij.m[2][2]);
//         }
//         eps = cvl_k - cvl_k_1;
//         cvl_k_1 = cvl_k;
//         /* Compute H[11] += I */
//         for(int n = 0; n < 3; n++) {
//             for(int m = 0; m < 3; m++) {
//                 H.coeffRef(n, m) += I.m[n][m];
//             }
//         }
//         H.makeCompressed();
//         /* Solving */
//         Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> chol(H); // Performs a Cholesky factorization of matrix H
//         if (chol.info() != Success) {
//             ROS_ERROR("Decomposition failed!");
//             failed = true;
//             break;
//         }else {
//             delta_x = chol.solve(-b);
//             if (chol.info() != Success) {
//                 ROS_ERROR("Solver failed!");
//                 failed = true;
//                 break;
//             }else {
//                 x += delta_x;
//                 num_inter += 1;
//             }
//         }   
//     }
//     /* H* = H; H*[11] -= I */
//     for(int n = 0; n < 3; n++) {
//         for(int m = 0; m < 3; m++) {
//             H.coeffRef(n, m) -= I.m[n][m];
//         }
//     }
//     if(!failed) {
//         optimized = true;
//         for(int k = 0; k < 3*num_nodes; k++) {
//             int i = k/3;
//             int j = k%3; 
//             graph_t_.set_node_t[i].pose.v[j] = x(k);
//         }
//     }
//     // cout << " x " << x << endl;
// }

// void cov_func(sl_graph_t& graph_t_) {
//     int num_nodes = graph_t_.set_node_t.size();
//     int num_edges = graph_t_.set_edge_t.size();

//     Eigen::VectorXd x(3*num_nodes);
//     Eigen::SparseMatrix<double> H(3*num_nodes, 3*num_nodes);
//     Eigen::SparseMatrix<double> H_red(3*(num_nodes -1), 3*(num_nodes - 1));

//     sl_matrix_t A_ij, B_ij, omega_ij;
//     sl_vector_t x_i, x_j, z_ij, e_ij;
    
//     sl_node_t node_i, node_j;
//     sl_edge_t edge_ij;

//     bool failed;
//     sl_matrix_t tmp_ij;
//     Eigen::MatrixXd I(3*(num_nodes -1), 3*(num_nodes - 1));
//     I.setIdentity();
//     /* Initial guess x[] */
//     for(int k = 0; k < 3*num_nodes; k++) {
//         int i = k/3; // node i_th
//         int j = k%3; // j = 0 -> x, j = 1 -> y, j = 2 -> theta
//         x(k) = graph_t_.set_node_t[i].pose.v[j];
//     }
//     H.setZero();
//     H_red.setZero();
//     for(int k = 0; k < num_nodes; k++) {
//         x(3*k+2) = normalize(x(3*k+2));
//     }
//     for(int k = 0; k < num_edges; k++) {
//         /* Edge ij from node i to node j*/
//         edge_ij = graph_t_.set_edge_t[k];
//         z_ij = edge_ij.z;

//         int i = edge_ij.i;
//         int j = edge_ij.j;

//         omega_ij = edge_ij.inv_cov;

//         x_i.v[0] = x(3*i); x_i.v[1] = x(3*i+1); x_i.v[2] = x(3*i+2);
//         x_j.v[0] = x(3*j); x_j.v[1] = x(3*j+1); x_j.v[2] = x(3*j+2);

//         jacobian_func(x_i, x_j, z_ij, A_ij, B_ij);
//         /* Compute H[ii] += A_ij^T*omega_ij*A_ij */
//         tmp_ij = trip_cross_matrix(A_ij, omega_ij);
//         for(int n = 3*i; n < 3*i+3; n++) {
//             for(int m = 3*i; m < 3*i+3; m++) {
//                 H.coeffRef(n, m) += tmp_ij.m[n%3][m%3];
//             }
//         }

//         /* Compute H[ij] += A_ij^T*omega_ij*B_ij */
//         tmp_ij = trip_cross_matrix(A_ij, omega_ij, B_ij);
//         for(int n = 3*i; n < 3*i+3; n++) {
//             for(int m = 3*j; m < 3*j+3; m++) {
//                 H.coeffRef(n, m) += tmp_ij.m[n%3][m%3];
//             }
//         }

//         /* Compute H[ji] += B_ij^T*omega_ij*A_ij */
//         tmp_ij = trip_cross_matrix(B_ij, omega_ij, A_ij);
//         for(int n = 3*j; n < 3*j+3; n++) {
//             for(int m = 3*i; m < 3*i+3; m++) {
//                 H.coeffRef(n, m) += tmp_ij.m[n%3][m%3];
//             }
//         }

//         /* Compute H[jj] += B_ij^T*omega_ij*B_ij */
//         tmp_ij = trip_cross_matrix(B_ij, omega_ij);
//         for(int n = 3*j; n < 3*j+3; n++) {
//             for(int m = 3*j; m < 3*j+3; m++) {
//                 H.coeffRef(n, m) += tmp_ij.m[n%3][m%3];
//             }
//         }
//     }
//     H.makeCompressed();
//     for(int i = 0; i < 3*(num_nodes - 1); i++) {
//         for(int j = 0; j < 3*(num_nodes - 1); j++) {
//             H_red.coeffRef(i, j) = H.coeffRef(i, j);
//         }
//     }
//     H_red.makeCompressed();
//     Eigen::MatrixXd H_red_inv;
//     Eigen::Matrix3d H_ii;
//     sl_matrix_t inv_cov_ii;
//     Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> chol(H_red);
//     if(chol.info()!= Success) {
//         ROS_INFO("H reduce inverse fail!");
//     }else {
//         H_red_inv = chol.solve(I);
//         for(int i = 0; i < num_nodes - 1; i++) {
//             H_ii = (H_red_inv.block(3*i, 3*i, 3, 3)).inverse();
//             for(int j = 0; j < 3; j++) {
//                 for(int k = 0; k < 3; k++) {
//                     inv_cov_ii.m[j][k] = H_ii(j, k);
//                 }
//             }
//             graph_t_.set_node_t[i].inv_cov = inv_cov_ii;
//         }
//     }
// }
