#pragma once
#include "vk_slam/vk_slam.hpp"

/** Checking loop closure */
bool check_loop_closure(sl_node_t& node_i, sl_node_t& node_j) {
    Eigen::Matrix2d R_ij;
    R_ij.setZero();
    sl_point_cloud_t pcl_ref, pcl_cur;
    compute_points(node_i, pcl_ref);
    compute_points(node_j, pcl_cur);
    int num_point = pcl_cur.pcl.size();
    sl_vector_t mean_ref, mean_cur;
    sl_vector_t trans;
    trans.v[0] = cos(node_i.pose.v[2])*(node_j.pose.v[0] - node_i.pose.v[0]) + sin(node_i.pose.v[2])*(node_j.pose.v[1] - node_i.pose.v[1]);
    trans.v[1] = -sin(node_i.pose.v[2])*(node_j.pose.v[0] - node_i.pose.v[0]) + cos(node_i.pose.v[2])*(node_j.pose.v[1] - node_i.pose.v[1]);
    trans.v[2] = angle_diff(node_j.pose.v[2], node_i.pose.v[2]);

    sl_point_cloud_t pcl_cur_w;
    vector<sl_corr_t> cores;
    Eigen::MatrixXd U, V, H_matrix;
    H_matrix.setZero(2, 2);

    double H[2][2];
    int count, num_cores;
    count = 0;
    double sum_e_k, sum_e_k_1, eps;
    sum_e_k_1 = inf; eps = inf;
    count = 0; num_cores = 0;
    double a = z_hit/sqrt(2*M_PI*pow(sigma, 2));
    transform_pcl(pcl_cur, pcl_cur_w, trans);
    while(fabs(eps) > 1e-3 && count < 0.2*max_inter_ICP) {
        get_correspondences(pcl_ref, pcl_cur_w, cores);
        compute_cov_mean_ICP(pcl_ref, pcl_cur, cores, mean_ref, mean_cur, H);
        for(int i = 0; i < 2; i++) {
            for(int j = 0; j < 2; j++) {
                H_matrix(i, j) = H[i][j];
            }
        }
        Eigen::JacobiSVD<MatrixXd> svd( H_matrix, ComputeThinU | ComputeThinV);
        U = svd.matrixU();
        V = svd.matrixV();
        R_ij = U*V.transpose();

        trans.v[2] = atan2(R_ij(1, 0), R_ij(0, 0));
        trans.v[0] = mean_ref.v[0] - (mean_cur.v[0]*cos(trans.v[2]) - mean_cur.v[1]*sin(trans.v[2]));
        trans.v[1] = mean_ref.v[1] - (mean_cur.v[0]*sin(trans.v[2]) + mean_cur.v[1]*cos(trans.v[2]));
        transform_pcl(pcl_cur, pcl_cur_w, trans);
        count += 1;
    }
    for(int i = 0; i < cores.size(); i++) {
        if(cores[i].weight > 0.8*a) {
            num_cores += 1;
        }
    }
    // cout << "match:" << (double)num_cores/pcl_cur.pcl.size()<< endl;
    if((double)num_cores/pcl_cur.pcl.size() >= match_rate_ICP) {
        return true;
    }
    return false;
}

double mahalanobis_distance(sl_node_t& node_i, sl_node_t& node_j) {
    sl_vector_t u;
    u.v[0] = node_j.pose.v[0] - node_i.pose.v[0];
    u.v[1] = node_j.pose.v[1] - node_i.pose.v[1];
    u.v[2] = angle_diff(node_j.pose.v[2], node_i.pose.v[2]);
    
    double d;
    d = (u.v[0]*node_i.inv_cov.m[0][0] + u.v[1]*node_i.inv_cov.m[1][0] + u.v[2]*node_i.inv_cov.m[2][0])*u.v[0];
    d += (u.v[0]*node_i.inv_cov.m[0][1] + u.v[1]*node_i.inv_cov.m[1][1] + u.v[2]*node_i.inv_cov.m[2][1])*u.v[1];
    d += (u.v[0]*node_i.inv_cov.m[0][2] + u.v[1]*node_i.inv_cov.m[1][2] + u.v[2]*node_i.inv_cov.m[2][2])*u.v[2];
    return sqrt(fabs(d));
}

void compute_loop_constraint(sl_node_t& node_i, sl_node_t& node_j, sl_edge_t& edge_ij) {
    Eigen::Matrix2d R_ij;
    R_ij.setZero();
    sl_point_cloud_t pcl_ref, pcl_cur;
    compute_points(node_i, pcl_ref);
    compute_points(node_j, pcl_cur);
    int num_point = pcl_cur.pcl.size();
    sl_vector_t mean_ref, mean_cur;
    sl_vector_t trans;
    trans.v[0] = cos(node_i.pose.v[2])*(node_j.pose.v[0] - node_i.pose.v[0]) + sin(node_i.pose.v[2])*(node_j.pose.v[1] - node_i.pose.v[1]);
    trans.v[1] = -sin(node_i.pose.v[2])*(node_j.pose.v[0] - node_i.pose.v[0]) + cos(node_i.pose.v[2])*(node_j.pose.v[1] - node_i.pose.v[1]);
    trans.v[2] = angle_diff(node_j.pose.v[2], node_i.pose.v[2]);

    sl_point_cloud_t pcl_cur_w;
    vector<sl_corr_t> cores;
    Eigen::MatrixXd U, V, H_matrix;
    H_matrix.setZero(2, 2);

    double H[2][2];
    int count, num_cores;
    double sum_e_k, sum_e_k_1, eps;
    sum_e_k_1 = inf; eps = inf;
    count = 0; num_cores = 0;
    transform_pcl(pcl_cur, pcl_cur_w, trans);
    while(fabs(eps) > 2e-4 && count < max_inter_ICP) {
        get_correspondences(pcl_ref, pcl_cur_w, cores);
        compute_cov_mean_ICP(pcl_ref, pcl_cur, cores, mean_ref, mean_cur, H);
        for(int i = 0; i < 2; i++) {
            for(int j = 0; j < 2; j++) {
                H_matrix(i, j) = H[i][j];
            }
        }
        Eigen::JacobiSVD<MatrixXd> svd( H_matrix, ComputeThinU | ComputeThinV);
        U = svd.matrixU();
        V = svd.matrixV();
        R_ij = U*V.transpose();

        trans.v[2] = atan2(R_ij(1, 0), R_ij(0, 0));
        trans.v[0] = mean_ref.v[0] - (mean_cur.v[0]*cos(trans.v[2]) - mean_cur.v[1]*sin(trans.v[2]));
        trans.v[1] = mean_ref.v[1] - (mean_cur.v[0]*sin(trans.v[2]) + mean_cur.v[1]*cos(trans.v[2]));
        transform_pcl(pcl_cur, pcl_cur_w, trans);
        sum_e_k = compute_sum_error_ICP(pcl_ref, pcl_cur_w, cores);

        eps = sum_e_k - sum_e_k_1;
        sum_e_k_1 = sum_e_k;
        count += 1;
    }
    edge_ij.i = node_i.idx;
    edge_ij.j = node_j.idx;
    edge_ij.z.v[0] = trans.v[0];
    edge_ij.z.v[1] = trans.v[1];
    edge_ij.z.v[2] = trans.v[2];
}

/** Detect loop-closure */
void detect_loop_closure(sl_graph_t& graph_t_) {
    int num_nodes = graph_t_.set_node_t.size();
    sl_node_t node_t = graph_t_.set_node_t.back();
    sl_edge_t edge_ij;
    double d;
    cov_func(graph_t_);
    int count = 0;
    for(int i = 0; i < num_nodes-1; i++) {
        d = mahalanobis_distance(graph_t_.set_node_t[i], node_t);
        if(d <= 3) {
            count += 1;
            overlap_best = check_loop_closure(graph_t_.set_node_t[i], node_t);
            if(overlap_best) {
                ROS_INFO("Loop closure detected! %d - %d",node_t.idx, i);
                loop_closure_detected = true;
                compute_loop_constraint(graph_t_.set_node_t[i], node_t, edge_ij);
                graph_t_.set_edge_t.push_back(edge_ij);
            }
        }
    }
}

void thread_func(sl_graph_t& graph_t_, vector<double>& log_map_t, nav_msgs::OccupancyGrid& map_t) {
    while(1) {
        loop_closure_detected = false;
        optimized = false;
        detect_loop_closure(graph_t_);
        if(loop_closure_detected) {
            optimization(graph_t_);
            ROS_INFO("Optimization done!");
            if(optimized) {
                mapping(graph_t_, log_map_t, map_t);
            }
        }
    }
}