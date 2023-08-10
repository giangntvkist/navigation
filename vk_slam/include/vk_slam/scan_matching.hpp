#pragma once
#include "vk_slam/vk_slam.hpp"

/** Pointcould in frame node i */
void compute_points(sl_node_t& node_i, sl_point_cloud_t& pcl_cur) {
    int num_points = node_i.scan.ranges.size();
    double r, beam_angle;
    sl_vector_t p;
    pcl_cur.pcl.clear();
    for(int k = 0; k < num_points; k++) {
        r = node_i.scan.ranges[k].v[0];
        beam_angle = node_i.scan.ranges[k].v[1] + laser_pose_theta;
        p.v[0] = laser_pose_x + r*cos(beam_angle);
        p.v[1] = laser_pose_y + r*sin(beam_angle);
        p.v[2] = beam_angle;
        pcl_cur.pcl.push_back(p);
    }
}

void transform_pcl(sl_point_cloud_t& pcl_cur, sl_point_cloud_t& pcl_cur_w, sl_vector_t& trans) {
    int num_point = pcl_cur.pcl.size();
    sl_vector_t p, p_w;
    pcl_cur_w.pcl.clear();
    for(int i = 0; i < num_point; i++) {
        p = pcl_cur.pcl[i];
        p_w.v[0] = p.v[0]*cos(trans.v[2]) - p.v[1]*sin(trans.v[2]) + trans.v[0];
        p_w.v[1] = p.v[0]*sin(trans.v[2]) + p.v[1]*cos(trans.v[2]) + trans.v[1];
        p_w.v[2] = p.v[2] + trans.v[2];
        pcl_cur_w.pcl.push_back(p_w);
    }
}

void get_correspondences(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur_w, vector<sl_corr_t>& cores) {
    int num_point_ref = pcl_ref.pcl.size();
    int num_point_cur = pcl_cur_w.pcl.size();
    double a = z_hit/sqrt(2*M_PI*pow(sigma, 2));
    double d_min, d;
    int idx;
    sl_vector_t p, q;
    sl_corr_t w;
    cores.clear();
    for(int i = 0; i < num_point_cur; i++) {
        d_min = inf;
        idx = -1;
        p = pcl_cur_w.pcl[i];
        for(int j = 0; j < num_point_ref; j++) {
            q = pcl_ref.pcl[j];
            d = pow(p.v[0] - q.v[0], 2) + pow(p.v[1] - q.v[1], 2);
            if(d < d_min) {
                d_min = d;
                idx = j;
            }
        }
        w.i = i;
        w.j1 = idx;
        w.dist2_j1 = d_min;
        w.weight = a*exp(-0.5*d_min/pow(sigma, 2));
        cores.push_back(w);
    }
}

void compute_cov_mean_ICP(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur, vector<sl_corr_t>& cores,
    sl_vector_t& mean_ref, sl_vector_t& mean_cur, double (&H)[2][2]) {
    int num_pairs = cores.size();
    double sum_weight = 0;
    int j;
    mean_ref.v[0] = 0; mean_ref.v[1] = 0;
    mean_cur.v[0] = 0; mean_cur.v[1] = 0;
    for(int i = 0; i < num_pairs; i++) {
        j = cores[i].j1;
        mean_ref.v[0] += cores[i].weight*pcl_ref.pcl[j].v[0];
        mean_ref.v[1] += cores[i].weight*pcl_ref.pcl[j].v[1];

        mean_cur.v[0] += cores[i].weight*pcl_cur.pcl[i].v[0];
        mean_cur.v[1] += cores[i].weight*pcl_cur.pcl[i].v[1];
        sum_weight += cores[i].weight;
    }
    mean_ref.v[0] = mean_ref.v[0]/sum_weight;
    mean_ref.v[1] = mean_ref.v[1]/sum_weight;

    mean_cur.v[0] = mean_cur.v[0]/sum_weight;
    mean_cur.v[1] = mean_cur.v[1]/sum_weight;
    for(int i = 0; i < 2; i++) {
        for(int j = 0; j < 2; j++) {
            H[i][j] = 0;
        }
    }
    for(int i = 0; i < num_pairs; i++) {
        j = cores[i].j1;
        H[0][0] += (pcl_ref.pcl[j].v[0] - mean_ref.v[0])*(pcl_cur.pcl[i].v[0] - mean_cur.v[0])*cores[i].weight;
        H[0][1] += (pcl_ref.pcl[j].v[0] - mean_ref.v[0])*(pcl_cur.pcl[i].v[1] - mean_cur.v[1])*cores[i].weight;

        H[1][0] += (pcl_ref.pcl[j].v[1] - mean_ref.v[1])*(pcl_cur.pcl[i].v[0] - mean_cur.v[0])*cores[i].weight;
        H[1][1] += (pcl_ref.pcl[j].v[1] - mean_ref.v[1])*(pcl_cur.pcl[i].v[1] - mean_cur.v[1])*cores[i].weight;
    }
}

double compute_sum_error_ICP(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur_w, vector<sl_corr_t>& cores) {
    double sum_e = 0;
    int num_point = pcl_cur_w.pcl.size();
    int j;
    for(int i = 0; i < num_point; i++) {
        j = cores[i].j1;
        sum_e += cores[i].weight*(pow(pcl_ref.pcl[j].v[0] - pcl_cur_w.pcl[i].v[0], 2) + pow(pcl_ref.pcl[j].v[1] - pcl_cur_w.pcl[i].v[1], 2));
    }
    return sum_e;
}

/** Scan matching ICP point-to-point */

void vanilla_ICP(sl_node_t& node_i, sl_node_t& node_j, sl_edge_t& edge_ij) {
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
    /* Caculate number of correspondence points */
    // double a = z_hit/sqrt(2*M_PI*pow(sigma, 2));
    // for(int i = 0; i < cores.size(); i++) {
    //     if(cores[i].weight > 0.95*a) {
    //         num_cores += 1;
    //     }
    // }
    // cout << "count: " << count << " " << sum_e_k << " " << num_cores << endl;
    if(count == max_inter_ICP) {
        ROS_WARN("Scan matching failed!");
        trans.v[0] = cos(node_i.pose.v[2])*(node_j.pose.v[0] - node_i.pose.v[0]) + sin(node_i.pose.v[2])*(node_j.pose.v[1] - node_i.pose.v[1]);
        trans.v[1] = -sin(node_i.pose.v[2])*(node_j.pose.v[0] - node_i.pose.v[0]) + cos(node_i.pose.v[2])*(node_j.pose.v[1] - node_i.pose.v[1]);
        trans.v[2] = angle_diff(node_j.pose.v[2], node_i.pose.v[2]);
    }else {
        node_j.pose.v[0] = trans.v[0]*cos(node_i.pose.v[2]) - trans.v[1]*sin(node_i.pose.v[2]) + node_i.pose.v[0];
        node_j.pose.v[1] = trans.v[0]*sin(node_i.pose.v[2]) + trans.v[1]*cos(node_i.pose.v[2]) + node_i.pose.v[1];
        node_j.pose.v[2] = trans.v[2] + node_i.pose.v[2];
    }
    edge_ij.i = node_i.idx;
    edge_ij.j = node_j.idx;
    edge_ij.z.v[0] = trans.v[0];
    edge_ij.z.v[1] = trans.v[1];
    edge_ij.z.v[2] = trans.v[2];
    inverse_cov_ICP(pcl_ref, pcl_cur, trans, edge_ij.inv_cov, num_cores);
}

double compute_sum_error_ICP_plus(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur, sl_vector_t& trans) {
    sl_point_cloud_t pcl_cur_w_;
    vector<sl_corr_t> cores_;
    transform_pcl(pcl_cur, pcl_cur_w_, trans);
    get_correspondences(pcl_ref, pcl_cur_w_, cores_);
    return compute_sum_error_ICP(pcl_ref, pcl_cur_w_, cores_);
}

void inverse_cov_ICP(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur, sl_vector_t& trans, sl_matrix_t& inv_cov_matrix, int num_cores) {
    double J_plus, J_diff, J;
    double J1, J2, J3, J4;
    /* H_xx */
    J = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans);
    double anpha = 0.5*(num_cores - 3)/J;

    sl_vector_t trans_plus, trans_diff;
    trans_plus = trans; trans_diff = trans;
    trans_plus.v[0] += 2*delta_x; trans_diff.v[0] -= 2*delta_x;
    J_plus = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_plus);
    J_diff = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_diff);
    inv_cov_matrix.m[0][0] = anpha*(J_plus - 2*J + J_diff)/(4*delta_x*delta_x);

    /* H_yy */
    trans_plus = trans; trans_diff = trans;
    trans_plus.v[1] += 2*delta_y; trans_diff.v[1] -= 2*delta_y;
    J_plus = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_plus);
    J_diff = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_diff);
    inv_cov_matrix.m[1][1] = anpha*(J_plus - 2*J + J_diff)/(4*delta_y*delta_y);

    /* H_theta_theta */
    trans_plus = trans; trans_diff = trans;
    trans_plus.v[1] += 2*delta_theta; trans_diff.v[1] -= 2*delta_theta;
    inv_cov_matrix.m[2][2] = anpha*(J_plus - 2*J + J_diff)/(4*delta_theta*delta_theta);

    /* H_xy = H_yx */
    sl_vector_t trans_pp, trans_pd, trans_dp, trans_dd;
    trans_pp = trans; trans_pd = trans; trans_dp = trans; trans_dd = trans;
    trans_pp.v[0] += delta_x; trans_pp.v[1] += delta_y;
    trans_pd.v[0] += delta_x; trans_pd.v[1] -= delta_y;
    trans_dp.v[0] -= delta_x; trans_dp.v[1] += delta_y;
    trans_dd.v[0] -= delta_x; trans_dd.v[1] -= delta_y;
    J1 = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_pp);
    J2 = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_pd);
    J3 = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_dp);
    J4 = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_dd);
    inv_cov_matrix.m[0][1] = anpha*(J1 - J2 - J3 + J4)/(4*delta_x*delta_y);
    inv_cov_matrix.m[1][0] = inv_cov_matrix.m[0][1];

    /* H_xtheta = H_thetax */
    trans_pp = trans; trans_pd = trans; trans_dp = trans; trans_dd = trans;
    trans_pp.v[0] += delta_x; trans_pp.v[1] += delta_theta;
    trans_pd.v[0] += delta_x; trans_pd.v[1] -= delta_theta;
    trans_dp.v[0] -= delta_x; trans_dp.v[1] += delta_theta;
    trans_dd.v[0] -= delta_x; trans_dd.v[1] -= delta_theta;
    J1 = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_pp);
    J2 = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_pd);
    J3 = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_dp);
    J4 = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_dd);
    inv_cov_matrix.m[0][2] = anpha*(J1 - J2 - J3 + J4)/(4*delta_x*delta_theta);
    inv_cov_matrix.m[2][0] = inv_cov_matrix.m[0][2];

    /* H_ytheta = H_thetay */
    trans_pp = trans; trans_pd = trans; trans_dp = trans; trans_dd = trans;
    trans_pp.v[0] += delta_y; trans_pp.v[1] += delta_theta;
    trans_pd.v[0] += delta_y; trans_pd.v[1] -= delta_theta;
    trans_dp.v[0] -= delta_y; trans_dp.v[1] += delta_theta;
    trans_dd.v[0] -= delta_y; trans_dd.v[1] -= delta_theta;
    J1 = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_pp);
    J2 = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_pd);
    J3 = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_dp);
    J4 = compute_sum_error_ICP_plus(pcl_ref, pcl_cur, trans_dd);
    inv_cov_matrix.m[1][2] = anpha*(J1 - J2 - J3 + J4)/(4*delta_y*delta_theta);
    inv_cov_matrix.m[2][1] = inv_cov_matrix.m[1][2];

    // printf_matrix(inv_cov_matrix);
}
