#pragma once
#include "vk_slam/vk_slam.hpp"

/** Checking loop closure */
void corr_scan_to_scan(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur_w, vector<sl_corr_t>& cores) {
    int num_point_ref = pcl_ref.pcl.size();
    int num_point_cur = pcl_cur_w.pcl.size();
    double d_min, d;
    int id_cores;
    sl_vector_t p, q;
    sl_corr_t w;
    bool new_core;

    cores.clear();
    for(int i = 0; i < num_point_cur; i++) {
        d_min = my_inf;
        id_cores = -1;
        p = pcl_cur_w.pcl[i];
        for(int j = 0; j < num_point_ref; j++) {
            q = pcl_ref.pcl[j];
            d = pow(p.v[0] - q.v[0], 2) + pow(p.v[1] - q.v[1], 2);
            if(d < d_min) {
                d_min = d;
                id_cores = j;
            }
        }
        new_core = true;
        for(int k = 0; k < cores.size(); k++) {
            if(cores[k].j == id_cores) {
                new_core = false;
                if(d_min < cores[k].dist2) {
                    cores[k].i = i;
                    cores[k].dist2 = d_min;
                }
                break;
            }
        }
        if(new_core && d_min < dist_threshold) {
            w.i = i;
            w.j = id_cores;
            w.dist2 = d_min;
            cores.push_back(w);
        }
    }
}

double sum_error(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur_w, vector<sl_corr_t>& cores) {
    double sum_e = 0;
    int num_pairs = cores.size();
    int j, k;
    for(int i = 0; i < num_pairs; i++) {
        k = cores[i].i;
        j = cores[i].j;
        sum_e += pow(pcl_ref.pcl[j].v[0] - pcl_cur_w.pcl[k].v[0], 2) + pow(pcl_ref.pcl[j].v[1] - pcl_cur_w.pcl[k].v[1], 2);
    }
    return sum_e;
}

void covariance_mean(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur, vector<sl_corr_t>& cores,
    sl_vector_t& mean_ref, sl_vector_t& mean_cur, double (&H)[2][2]) {
    int num_pairs = cores.size();
    int j, k;
    mean_ref.v[0] = 0; mean_ref.v[1] = 0;
    mean_cur.v[0] = 0; mean_cur.v[1] = 0;
    for(int i = 0; i < num_pairs; i++) {
        k = cores[i].i;
        j = cores[i].j;
        mean_ref.v[0] += pcl_ref.pcl[j].v[0];
        mean_ref.v[1] += pcl_ref.pcl[j].v[1];

        mean_cur.v[0] += pcl_cur.pcl[k].v[0];
        mean_cur.v[1] += pcl_cur.pcl[k].v[1];
    }
    mean_ref.v[0] = mean_ref.v[0]/num_pairs;
    mean_ref.v[1] = mean_ref.v[1]/num_pairs;

    mean_cur.v[0] = mean_cur.v[0]/num_pairs;
    mean_cur.v[1] = mean_cur.v[1]/num_pairs;
    for(int i = 0; i < 2; i++) {
        for(int j = 0; j < 2; j++) {
            H[i][j] = 0;
        }
    }
    for(int i = 0; i < num_pairs; i++) {
        k = cores[i].i;
        j = cores[i].j;
        H[0][0] += (pcl_ref.pcl[j].v[0] - mean_ref.v[0])*(pcl_cur.pcl[k].v[0] - mean_cur.v[0]);
        H[0][1] += (pcl_ref.pcl[j].v[0] - mean_ref.v[0])*(pcl_cur.pcl[k].v[1] - mean_cur.v[1]);

        H[1][0] += (pcl_ref.pcl[j].v[1] - mean_ref.v[1])*(pcl_cur.pcl[k].v[0] - mean_cur.v[0]);
        H[1][1] += (pcl_ref.pcl[j].v[1] - mean_ref.v[1])*(pcl_cur.pcl[k].v[1] - mean_cur.v[1]);
    }
}

bool check_loop_closure(sl_node_t& node_i, sl_node_t& node_j, sl_edge_t& edge_ij) {
    Eigen::Matrix2d R;
    R.setZero();
    sl_point_cloud_t pcl_ref, pcl_cur, pcl_cur_w;
    compute_points(node_i, pcl_ref);
    compute_points(node_j, pcl_cur);
    sl_vector_t mean_ref, mean_cur, trans;

    trans.v[0] = cos(node_i.pose.v[2])*(node_j.pose.v[0] - node_i.pose.v[0]) + sin(node_i.pose.v[2])*(node_j.pose.v[1] - node_i.pose.v[1]);
    trans.v[1] = -sin(node_i.pose.v[2])*(node_j.pose.v[0] - node_i.pose.v[0]) + cos(node_i.pose.v[2])*(node_j.pose.v[1] - node_i.pose.v[1]);
    trans.v[2] = angle_diff(node_j.pose.v[2], node_i.pose.v[2]);

    vector<sl_corr_t> cores;
    Eigen::MatrixXd U, V, H_matrix;
    H_matrix.setZero(2, 2);
    double H[2][2];
    int count, num_cores;
    double sum_e_k, sum_e_k_1, eps;
    count = 0; eps = my_inf;
    transform_pcl(pcl_cur, pcl_cur_w, trans);
    corr_scan_to_scan(pcl_ref, pcl_cur_w, cores);
    sum_e_k_1 = sum_error(pcl_ref, pcl_cur_w, cores);
    while(eps > 2e-4 && count < max_inter) {
        covariance_mean(pcl_ref, pcl_cur_w, cores, mean_ref, mean_cur, H);
        for(int i = 0; i < 2; i++) {
            for(int j = 0; j < 2; j++) {
                H_matrix(i, j) = H[i][j];
            }
        }
        Eigen::JacobiSVD<MatrixXd> svd( H_matrix, ComputeThinU | ComputeThinV);
        U = svd.matrixU();
        V = svd.matrixV();
        R = U*V.transpose();

        trans.v[2] = atan2(R(1, 0), R(0, 0));
        trans.v[0] = mean_ref.v[0] - (mean_cur.v[0]*cos(trans.v[2]) - mean_cur.v[1]*sin(trans.v[2]));
        trans.v[1] = mean_ref.v[1] - (mean_cur.v[0]*sin(trans.v[2]) + mean_cur.v[1]*cos(trans.v[2]));
        transform_pcl(pcl_cur, pcl_cur_w, trans);
        corr_scan_to_scan(pcl_ref, pcl_cur_w, cores);
        num_cores = cores.size();
        sum_e_k = sum_error(pcl_ref, pcl_cur_w, cores);

        eps = fabs(sum_e_k - sum_e_k_1);
        sum_e_k_1 = sum_e_k;
        count += 1;
    }
    if((double)num_cores/pcl_cur.pcl.size() > match_rate && count < max_inter && sum_e_k < 5e-4*num_cores) {
        edge_ij.i = node_i.idx;
        edge_ij.j = node_j.idx;
        edge_ij.z.v[0] = trans.v[0];
        edge_ij.z.v[1] = trans.v[1];
        edge_ij.z.v[2] = trans.v[2];
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

/** Detect loop-closure */
void detect_loop_closure(sl_graph_t& graph_t_) {
    int num_nodes = graph_t_.set_node_t.size();
    sl_node_t node_t = graph_t_.set_node_t.back();
    sl_edge_t edge_ij;
    double d;
    cov_func(graph_t_);
    for(int i = 0; i < num_nodes-1; i++) {
        d = mahalanobis_distance(graph_t_.set_node_t[i], node_t);
        if(d <= 3) {
            overlap_best = check_loop_closure(graph_t_.set_node_t[i], node_t, edge_ij);
            if(overlap_best) {
                ROS_INFO("Loop closure detected! %d - %d",node_t.idx, i);
                loop_closure_detected = true;
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