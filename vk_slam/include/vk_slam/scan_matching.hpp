#pragma once
#include "vk_slam/vk_slam.hpp"

/** Pointcould in frame node i */
Eigen::MatrixXd compute_points(sl_node_t& node_i) {
    int num_points = node_i.scan.ranges.size();
    Eigen::MatrixXd pcl(2, num_points);
    pcl.setZero();
    double r, beam_angle;
    for(int k = 0; k < num_points; k++) {
        r = node_i.scan.ranges[k].v[0];
        beam_angle = node_i.scan.ranges[k].v[1] + laser_pose_theta;
        pcl(0, k) = laser_pose_x + r*cos(beam_angle);
        pcl(1, k) = laser_pose_y + r*sin(beam_angle);
    }
    return pcl;
}

double norm2(Eigen::Vector2d& p, Eigen::Vector2d& q) {
    return pow(p(0) - q(0), 2) + pow(p(1) - q(1), 2);
}

/** Get correspondences refscan: A & curscan: B
    - Matrix A: 2xN, N is number of points reference scan
    - Matrix B: 2xM, M is number of points current scan
    - Matrix cores: 4xM, cores(0, i): index of point ith B, cores(1, i): index of point jth A which point is closest with point ith
    cores(2, i): distance minimum between point ith(B) and jth(A), cores(3, i): weight of pair ij */
void get_correspondences(Eigen::MatrixXd& cores, Eigen::MatrixXd& A, Eigen::MatrixXd& B) {
    int num_pointA = A.cols();
    int num_pointB = B.cols();
    cores.setZero(4, num_pointB);
    double d_min, d;
    int idx;
    Eigen::Vector2d p, q;
    for(int i = 0; i < num_pointB; i++) {
        d_min = inf;
        idx = -1;
        for(int j = 0; j < num_pointA; j++) {
            p = B.col(i);
            q = A.col(j);
            d = norm2(p, q);
            if(d < d_min) {
                d_min = d;
                idx = j;
            }
        }
        if(d_min < dist_threshold) {
            cores(0, i) = i;
            cores(1, i) = idx;
            cores(2, i) = d_min;
            cores(3, i) = 1;
        }else {
            /** Reject oulier larger distance threshold */
            cores(0, i) = i;
            cores(1, i) = -1;
            cores(2, i) = d_min;
            cores(3, i) = 1;
        }
    }
    for(int i = 0; i < num_pointB; i++) {
        for(int j = 0; j < num_pointB; j++) {
            if(i != j && cores(1, i) != -1) {
                if(cores(1, i) == cores(1, j)) {
                    if(cores(2, i) <= cores(2, j)) {
                        cores(1, j) = -1;
                    }else {
                        cores(1, i) = -1;
                    }
                }
            }
        }
    }
}

/** Compute cross-Covariance matrix H and Mean value */
void crossCovariance_Mean(Eigen::MatrixXd& cores, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::Vector2d& mean_A, Eigen::Vector2d& mean_B, Eigen::MatrixXd& H) {
    mean_A.setZero(2, 1);
    mean_B.setZero(2, 1);
    int num_pairs = cores.cols();
    double sum_weight = 0;
    for(int i = 0; i < num_pairs; i++) {
        int j = cores(1, i);
        if(j != -1) {
            mean_A += cores(3, i)*A.col(j);
            mean_B += cores(3, i)*B.col(i);
            sum_weight += cores(3, i);
        }
    }
    mean_A = mean_A/sum_weight;
    mean_B = mean_B/sum_weight;

    H.setZero(2, 2);
    for(int i = 0; i < num_pairs; i++) {
        int j = cores(1, i);
        if(j != -1) {
            H += (A.col(j) - mean_A)*(B.col(i) - mean_B).transpose()*cores(3, i);
        }
    }
}

double error_ICP(Eigen::MatrixXd& cores, Eigen::MatrixXd& pcl_ref, Eigen::MatrixXd& pcl_cur, Eigen::Matrix2d& R, Eigen::Vector2d& t) {
    double e = 0.0;
    int num_point = pcl_cur.cols();
    Eigen::Vector2d p;
    Eigen::Vector2d q;
    for(int i = 0; i < num_point; i++) {
        int j = cores(1, i);
        if(j != -1) {
            p = R*pcl_cur.col(i) + t;
            q = pcl_ref.col(cores(1, i));
            e += norm2(p, q);
        }
    }
    return e;
}

double error_ICP_plus(Eigen::MatrixXd& pcl_ref, Eigen::MatrixXd& pcl_cur, Eigen::Matrix2d& R, Eigen::Vector2d& t) {
    double e = 0.0;
    int num_point = pcl_cur.cols();
    Eigen::MatrixXd pcl_temp;
    pcl_temp.setZero(2, num_point);
    for(int k = 0; k < num_point; k++) {
        pcl_temp.col(k) = R*pcl_cur.col(k) + t;
    }
    Eigen::MatrixXd cores;
    get_correspondences(cores, pcl_ref, pcl_temp);
    Eigen::Vector2d p, q;
    for(int i = 0; i < num_point; i++) {
        int j = cores(1, i);
        if(j != -1) {
            p = pcl_temp.col(i);
            q = pcl_ref.col(cores(1, i));
            e += norm2(p, q);
        }
    }
    return e;
}

/** Vanilla ICP algorithm */
void vanilla_ICP(sl_node_t& node_i, sl_node_t& node_j, sl_edge_t& edge_ij) {
    Eigen::Matrix2d R_i, R_j, R_ij;
    Eigen::Vector2d t_i, t_j, t_ij;
    double theta_i, theta_j;
    theta_i = node_i.pose.v[2];
    theta_j = node_j.pose.v[2];
    R_i << cos(theta_i), -sin(theta_i),
        sin(theta_i), cos(theta_i);
        
    t_i << node_i.pose.v[0],
        node_i.pose.v[1];

    R_j << cos(theta_j), -sin(theta_j),
        sin(theta_j), cos(theta_j);

    t_j << node_j.pose.v[0],
        node_j.pose.v[1];

    R_ij = R_i.transpose()*R_j;
    t_ij = R_i.transpose()*(t_j - t_i);

    Eigen::MatrixXd pcl_ref, pcl_cur;
    pcl_ref = compute_points(node_i);
    pcl_cur = compute_points(node_j);
    Eigen::MatrixXd cores, pcl_temp;
    int num_pointcur = pcl_cur.cols();
    pcl_temp.setZero(2, num_pointcur);
    Eigen::Vector2d mean_A;
    Eigen::Vector2d mean_B;
    Eigen::MatrixXd H;
    Eigen::MatrixXd U, V;
    int num_inter = 0;
    double e_k, e_k_1, eps;
    eps = inf;
    e_k_1 = inf;
    while(fabs(eps) > converged_graph && num_inter < max_inter) {
        for(int k = 0; k < num_pointcur; k++) {
            pcl_temp.col(k) = R_ij*pcl_cur.col(k) + t_ij;
        }
        get_correspondences(cores, pcl_ref, pcl_temp);
        crossCovariance_Mean(cores, pcl_ref, pcl_cur, mean_A, mean_B, H);

        Eigen::JacobiSVD<MatrixXd> svd( H, ComputeThinU | ComputeThinV);
        U = svd.matrixU();
        V = svd.matrixV();
        R_ij = U*V.transpose();
        t_ij = mean_A - R_ij*mean_B;
        e_k = error_ICP(cores, pcl_ref, pcl_cur, R_ij, t_ij);
        eps = e_k - e_k_1;
        e_k_1 = e_k;
        num_inter += 1;
    }
    if(num_inter == max_inter) {
        ROS_WARN("ICP don't coverge!");
        max_inter_ICP = true;
        R_ij = R_i.transpose()*R_j;
        t_ij = R_i.transpose()*(t_j - t_i);
    }else {
        max_inter_ICP = false;
        if(e_k <= e_threshold) {
        	scan_matching_success = true;
    	}
    	cout << " Error: " << e_k << endl;
    }
    /** Update node j*/
    t_j = R_i*t_ij + t_i;
    double theta_ij = atan2(R_ij(1, 0), R_ij(0, 0));
    node_j.pose.v[0] = t_j(0);
    node_j.pose.v[1] = t_j(1);
    node_j.pose.v[2] = theta_ij + theta_i;

    /** Add new constraint node i to node j*/
    edge_ij.i = node_i.idx;
    edge_ij.j = node_j.idx;
    edge_ij.z.v[0] = t_ij(0);
    edge_ij.z.v[1] = t_ij(1);
    edge_ij.z.v[2] = theta_ij;

    inv_covariance_ICP(pcl_ref, pcl_cur, R_ij, t_ij, edge_ij.inv_cov);
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            edge_ij.inv_cov.m[i][j] *= 2*pow(sigma, -2);
        }
    }
}

/** Inverse Covariance vanilla ICP cov(R-t) */
void inv_covariance_ICP(Eigen::MatrixXd& pcl_ref, Eigen::MatrixXd& pcl_cur, Eigen::Matrix2d& R, Eigen::Vector2d& t, sl_matrix_t& inv_cov_matrix) {
    double theta = atan2(R(1, 0), R(0, 0));
    Eigen::Matrix2d R_plus, R_diff;
    Eigen::Vector2d t_plus, t_diff;
    double J_plus, J, J_diff;
    J = error_ICP_plus(pcl_ref, pcl_cur, R, t);
    /* H_00 */
    t_plus << t(0) + 2*delta_x,
        t(1);
    t_diff << t(0) - 2*delta_x,
        t(1);

    J_plus = error_ICP_plus(pcl_ref, pcl_cur, R, t_plus);
    J_diff = error_ICP_plus(pcl_ref, pcl_cur, R, t_diff);
    inv_cov_matrix.m[0][0] = (J_plus - 2*J + J_diff)/(4*delta_x*delta_x);
    
    /* H_11 */
    t_plus(0) = t(0); t_plus(1) = t(1) + 2*delta_y;
    t_diff(0) = t(0); t_diff(1) = t(1) - 2*delta_y;

    J_plus = error_ICP_plus(pcl_ref, pcl_cur, R, t_plus);
    J_diff = error_ICP_plus(pcl_ref, pcl_cur, R, t_diff);
    inv_cov_matrix.m[1][1] = (J_plus - 2*J + J_diff)/(4*delta_y*delta_y);

    /* H_22*/
    R_plus << cos(theta + 2*delta_theta), -sin(theta + 2*delta_theta),
        sin(theta + 2*delta_theta), cos(theta + 2*delta_theta);
    R_diff << cos(theta - 2*delta_theta), -sin(theta - 2*delta_theta),
        sin(theta - 2*delta_theta), cos(theta - 2*delta_theta);

    J_plus = error_ICP_plus(pcl_ref, pcl_cur, R_plus, t);
    J_diff = error_ICP_plus(pcl_ref, pcl_cur, R_diff, t);
    inv_cov_matrix.m[2][2] = (J_plus - 2*J + J_diff)/(4*delta_theta*delta_theta);

    Eigen::Vector2d t1, t2, t3, t4;
    /* H_xy = H_yx */
    t1 << t(0) + delta_x,
        t(1) + delta_y;
    t2 << t(0) - delta_x,
        t(1) + delta_y;
    t3 << t(0) + delta_x,
        t(1) - delta_y;
    t4 << t(0) - delta_x,
        t(1) - delta_y;
    double J1, J2, J3, J4;
    J1 = error_ICP_plus(pcl_ref, pcl_cur, R, t1);
    J2 = error_ICP_plus(pcl_ref, pcl_cur, R, t2);
    J3 = error_ICP_plus(pcl_ref, pcl_cur, R, t3);
    J4 = error_ICP_plus(pcl_ref, pcl_cur, R, t4);
    inv_cov_matrix.m[0][1] = (J1 - J2 - J3 + J4)/(4*delta_x*delta_y);
    inv_cov_matrix.m[1][0] = inv_cov_matrix.m[0][1];

    /* H_xtheta = H_theta */
    t_plus << t(0) + delta_x,
        t(1);
    R_plus << cos(theta + delta_theta), -sin(theta + delta_theta),
        sin(theta + delta_theta), cos(theta + delta_theta);
    
    t_diff << t(0) - delta_x,
        t(1);
    R_diff << cos(theta - delta_theta), -sin(theta - delta_theta),
        sin(theta - delta_theta), cos(theta - delta_theta);
    J1 = error_ICP_plus(pcl_ref, pcl_cur, R_plus, t_plus);
    J2 = error_ICP_plus(pcl_ref, pcl_cur, R_plus, t_diff);
    J3 = error_ICP_plus(pcl_ref, pcl_cur, R_diff, t_plus);
    J4 = error_ICP_plus(pcl_ref, pcl_cur, R_diff, t_diff);
    inv_cov_matrix.m[0][2] = (J1 - J2 - J3 + J4)/(4*delta_x*delta_theta);
    inv_cov_matrix.m[2][0] = inv_cov_matrix.m[0][2];

    /* H_ytheta = H_thetay */
    t_plus << t(0),
        t(1) + delta_y;
    t_diff << t(0),
        t(1) - delta_y;
    
    J1 = error_ICP_plus(pcl_ref, pcl_cur, R_plus, t_plus);
    J2 = error_ICP_plus(pcl_ref, pcl_cur, R_plus, t_diff);
    J3 = error_ICP_plus(pcl_ref, pcl_cur, R_diff, t_plus);
    J4 = error_ICP_plus(pcl_ref, pcl_cur, R_diff, t_diff);
    inv_cov_matrix.m[1][2] = (J1 - J2 - J3 + J4)/(4*delta_y*delta_theta);
    inv_cov_matrix.m[2][1] = inv_cov_matrix.m[1][2];
}

/** Compute mahalanobis distance*/
double mahalanobis_distance(sl_node_t& node_t, sl_node_t& node_i) {
    Eigen::Matrix3d omega;
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            omega(i, j) = node_i.inv_cov.m[i][j];
        }
    }
    Eigen::Vector3d x, muy;
    x << node_t.pose.v[0],
        node_t.pose.v[1],
        node_t.pose.v[2];

    muy << node_i.pose.v[0],
        node_i.pose.v[1],
        node_i.pose.v[2];
    return sqrt((x - muy).transpose()*omega*(x - muy));
}

/** Detect loop-closure*/
void detect_loop_closure(double& cumulative_distance, sl_graph_t& graph_t_) {
    if(cumulative_distance > min_cumulative_distance) {
        ROS_INFO("Detecting loop clousre...");
        int num_nodes = graph_t_.set_node_t.size();
        sl_node_t node_t = graph_t_.set_node_t.back();
        sl_edge_t edge_ij;
        double d;
        cov_func(graph_t_);
        for(int i = 0; i < num_nodes-1; i++) {
            d = mahalanobis_distance(node_t, graph_t_.set_node_t[i]);
            if(d <= 3) {
                scan_matching_success = false;
                vanilla_ICP(graph_t_.set_node_t[i], node_t, edge_ij);
                if(scan_matching_success) {
                    loop_closure_detected = true;
                    ROS_INFO("Loop closure detected! %d - %d",node_t.idx, i);
                    graph_t_.set_edge_t.push_back(edge_ij);
                }
            }
        }
    }
}
