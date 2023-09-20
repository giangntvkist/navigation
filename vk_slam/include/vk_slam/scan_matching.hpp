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
        pcl_cur.pcl.push_back(p);
    }
}

/** Transform point cloud in frame map*/
void transform_pcl(sl_point_cloud_t& pcl_cur, sl_point_cloud_t& pcl_cur_w, sl_vector_t& trans) {
    int num_points = pcl_cur.pcl.size();
    sl_vector_t p, p_w;
    pcl_cur_w.pcl.clear();
    for(int i = 0; i < num_points; i++) {
        p = pcl_cur.pcl[i];
        p_w.v[0] = p.v[0]*cos(trans.v[2]) - p.v[1]*sin(trans.v[2]) + trans.v[0];
        p_w.v[1] = p.v[0]*sin(trans.v[2]) + p.v[1]*cos(trans.v[2]) + trans.v[1];
        pcl_cur_w.pcl.push_back(p_w);
    }
}

double norm2(sl_vector_t& p, sl_vector_t& q) {
    return pow(p.v[0] - q.v[0], 2) + pow(p.v[1] - q.v[1], 2);
}

/** Get correspondence cell*/
void correspondences(sl_point_cloud_t& pcl_cur_w, nav_msgs::OccupancyGrid& map_t, vector<sl_corr_t>& cores) {
    cores.clear();
    int num_points = pcl_cur_w.pcl.size();
    int idx, idy, id_cell, id_cores;
    sl_corr_t w;
    double d_min;
    sl_vector_t cell;
    bool new_core;
    for(int i = 0; i < num_points; i++) {
        idx = (pcl_cur_w.pcl[i].v[0] - map_t.info.origin.position.x)/map_t.info.resolution;
        idy = (pcl_cur_w.pcl[i].v[1] - map_t.info.origin.position.y)/map_t.info.resolution;
        id_cell = idy*map_t.info.width + idx;
        d_min = my_inf;
        id_cores = -1;
        for(int j = idy - kernel_size; j < idy + kernel_size; j++) {
            if(j >= 0 && j < map_t.info.height) {
                for(int k = idx - kernel_size; k < idx + kernel_size; k++) {
                    if(k >= 0 && k < map_t.info.width) {
                        if(map_t.data[j*map_t.info.width + k] == occupied) {
                            cell.v[0] = k*map_t.info.resolution + map_t.info.resolution/2 + map_t.info.origin.position.x;
                            cell.v[1] = j*map_t.info.resolution + map_t.info.resolution/2 + map_t.info.origin.position.y;
                            if(norm2(cell, pcl_cur_w.pcl[i]) < d_min) {
                                d_min = norm2(cell, pcl_cur_w.pcl[i]);
                                id_cores = j*map_t.info.width + k;
                            }
                        }
                    }
                }
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

void compute_cov_mean_ICP(sl_point_cloud_t& pcl_cur, nav_msgs::OccupancyGrid& map_t, vector<sl_corr_t>& cores,
    sl_vector_t& mean_ref, sl_vector_t& mean_cur, double (&H)[2][2]) {
    
    int num_pairs = cores.size();
    mean_ref.v[0] = 0; mean_ref.v[1] = 0;
    mean_cur.v[0] = 0; mean_cur.v[1] = 0;
    int j, k, idx, idy;
    for(int i = 0; i < num_pairs; i++) {
        k = cores[i].j;
        idy = k/map_t.info.width;
        idx = k - idy*map_t.info.width;
        mean_ref.v[0] += idx*map_t.info.resolution + map_t.info.resolution/2 + map_t.info.origin.position.x;
        mean_ref.v[1] += idy*map_t.info.resolution + map_t.info.resolution/2 + map_t.info.origin.position.y;

        j = cores[i].i;
        mean_cur.v[0] += pcl_cur.pcl[j].v[0];
        mean_cur.v[1] += pcl_cur.pcl[j].v[1];
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

    double x_cell, y_cell;
    for(int i = 0; i < num_pairs; i++) {
        k = cores[i].j;
        idy = k/map_t.info.width;
        idx = k - idy*map_t.info.width;
        x_cell = idx*map_t.info.resolution + map_t.info.resolution/2 + map_t.info.origin.position.x;
        y_cell = idy*map_t.info.resolution + map_t.info.resolution/2 + map_t.info.origin.position.y;

        j = cores[i].i;
        H[0][0] += (x_cell - mean_ref.v[0])*(pcl_cur.pcl[j].v[0] - mean_cur.v[0]);
        H[0][1] += (x_cell - mean_ref.v[0])*(pcl_cur.pcl[j].v[1] - mean_cur.v[1]);

        H[1][0] += (y_cell - mean_ref.v[1])*(pcl_cur.pcl[j].v[0] - mean_cur.v[0]);
        H[1][1] += (y_cell - mean_ref.v[1])*(pcl_cur.pcl[j].v[1] - mean_cur.v[1]);
    }
}

double compute_sum_error_ICP(nav_msgs::OccupancyGrid& map_t, sl_point_cloud_t& pcl_cur_w, vector<sl_corr_t>& cores) {
    double sum_e = 0;
    int num_pairs = cores.size();
    int j, k, idx, idy;
    sl_vector_t cell;
    for(int i = 0; i < num_pairs; i++) {
        k = cores[i].j;
        idy = k/map_t.info.width;
        idx = k - idy*map_t.info.width;
        cell.v[0] = idx*map_t.info.resolution + map_t.info.resolution/2 + map_t.info.origin.position.x;
        cell.v[1] = idy*map_t.info.resolution + map_t.info.resolution/2 + map_t.info.origin.position.y;

        j = cores[i].i;
        sum_e += norm2(cell, pcl_cur_w.pcl[j]);
    }
    return sum_e;
}

/** Scan matching ICP point-to-point */
void vanilla_ICP(sl_node_t& node_i, sl_node_t& node_j, sl_edge_t& edge_ij, nav_msgs::OccupancyGrid& map_t) {
    Eigen::Matrix2d R;
    R.setZero();
    sl_point_cloud_t pcl_cur;
    compute_points(node_j, pcl_cur);

    vector<sl_corr_t> cores;
    sl_point_cloud_t pcl_cur_w;
    Eigen::MatrixXd U, V, H_matrix;
    H_matrix.setZero(2, 2);

    sl_vector_t trans;
    trans = node_j.pose;

    sl_vector_t mean_ref, mean_cur;
    double H[2][2];
    int count, num_cores;
    double sum_e_k, sum_e_k_1, eps;
    count = 0; eps = my_inf;

    transform_pcl(pcl_cur, pcl_cur_w, trans);
    correspondences(pcl_cur_w, map_t, cores);
    sum_e_k_1 = compute_sum_error_ICP(map_t, pcl_cur_w, cores);
    while(eps > 1e-4 && count < max_inter) {
        compute_cov_mean_ICP(pcl_cur, map_t, cores, mean_ref, mean_cur, H);
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
        correspondences(pcl_cur_w, map_t, cores);
        num_cores = cores.size();
        sum_e_k = compute_sum_error_ICP(map_t, pcl_cur_w, cores);

        eps = fabs(sum_e_k - sum_e_k_1);
        sum_e_k_1 = sum_e_k;
        count += 1;
    }
    // cout << "num:" << num_cores << " " << pcl_cur.pcl.size() << " " << count << endl;
    if(count == max_inter) {
        ROS_WARN("Scan matching failed!");
    }else {
        node_j.pose = trans;
    }
    edge_ij.i = node_i.idx;
    edge_ij.j = node_j.idx;
    edge_ij.z.v[0] = cos(node_i.pose.v[2])*(node_j.pose.v[0] - node_i.pose.v[0]) + sin(node_i.pose.v[2])*(node_j.pose.v[1] - node_i.pose.v[1]);
    edge_ij.z.v[1] = -sin(node_i.pose.v[2])*(node_j.pose.v[0] - node_i.pose.v[0]) + cos(node_i.pose.v[2])*(node_j.pose.v[1] - node_i.pose.v[1]);
    edge_ij.z.v[2] = angle_diff(node_j.pose.v[2], node_i.pose.v[2]);
    inverse_cov_ICP(map_t, pcl_cur, trans, edge_ij.inv_cov, num_cores);
}

/** Compute inverse covariance matrix ICP */
double compute_sum_error_ICP_plus(nav_msgs::OccupancyGrid& map_t, sl_point_cloud_t& pcl_cur, sl_vector_t& trans) {
    sl_point_cloud_t pcl_cur_w;
    vector<sl_corr_t> cores;
    transform_pcl(pcl_cur, pcl_cur_w, trans);
    correspondences(pcl_cur_w, map_t, cores);
    return compute_sum_error_ICP(map_t, pcl_cur_w, cores);
}

void inverse_cov_ICP(nav_msgs::OccupancyGrid& map_t, sl_point_cloud_t& pcl_cur, sl_vector_t& trans, sl_matrix_t& inv_cov_matrix, int num_cores) {
    double J_plus, J_diff, J;
    double J1, J2, J3, J4;
    /* H_xx */
    J = compute_sum_error_ICP_plus(map_t, pcl_cur, trans);
    // double anpha = 0.5*(num_cores - 3)/J;
    double anpha = 1.0;

    sl_vector_t trans_plus, trans_diff;
    trans_plus = trans; trans_diff = trans;
    trans_plus.v[0] += 2*delta_x; trans_diff.v[0] -= 2*delta_x;
    J_plus = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_plus);
    J_diff = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_diff);
    inv_cov_matrix.m[0][0] = anpha*(J_plus - 2*J + J_diff)/(4*delta_x*delta_x);

    /* H_yy */
    trans_plus = trans; trans_diff = trans;
    trans_plus.v[1] += 2*delta_y; trans_diff.v[1] -= 2*delta_y;
    J_plus = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_plus);
    J_diff = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_diff);
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
    J1 = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_pp);
    J2 = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_pd);
    J3 = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_dp);
    J4 = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_dd);
    inv_cov_matrix.m[0][1] = anpha*(J1 - J2 - J3 + J4)/(4*delta_x*delta_y);
    inv_cov_matrix.m[1][0] = inv_cov_matrix.m[0][1];

    /* H_xtheta = H_thetax */
    trans_pp = trans; trans_pd = trans; trans_dp = trans; trans_dd = trans;
    trans_pp.v[0] += delta_x; trans_pp.v[1] += delta_theta;
    trans_pd.v[0] += delta_x; trans_pd.v[1] -= delta_theta;
    trans_dp.v[0] -= delta_x; trans_dp.v[1] += delta_theta;
    trans_dd.v[0] -= delta_x; trans_dd.v[1] -= delta_theta;
    J1 = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_pp);
    J2 = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_pd);
    J3 = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_dp);
    J4 = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_dd);
    inv_cov_matrix.m[0][2] = anpha*(J1 - J2 - J3 + J4)/(4*delta_x*delta_theta);
    inv_cov_matrix.m[2][0] = inv_cov_matrix.m[0][2];

    /* H_ytheta = H_thetay */
    trans_pp = trans; trans_pd = trans; trans_dp = trans; trans_dd = trans;
    trans_pp.v[0] += delta_y; trans_pp.v[1] += delta_theta;
    trans_pd.v[0] += delta_y; trans_pd.v[1] -= delta_theta;
    trans_dp.v[0] -= delta_y; trans_dp.v[1] += delta_theta;
    trans_dd.v[0] -= delta_y; trans_dd.v[1] -= delta_theta;
    J1 = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_pp);
    J2 = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_pd);
    J3 = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_dp);
    J4 = compute_sum_error_ICP_plus(map_t, pcl_cur, trans_dd);
    inv_cov_matrix.m[1][2] = anpha*(J1 - J2 - J3 + J4)/(4*delta_y*delta_theta);
    inv_cov_matrix.m[2][1] = inv_cov_matrix.m[1][2];
    // printf_matrix(inv_cov_matrix);
}