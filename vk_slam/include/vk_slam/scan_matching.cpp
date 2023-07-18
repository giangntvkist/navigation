#pragma once
#include "vk_slam/vk_slam.hpp"

bool update_node(sl_vector_t u_t[2]) {
    return sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2)) > min_trans || angle_diff(u_t[1].v[2], u_t[0].v[2]) > min_rot;
}

void update_motion(sl_vector_t u_t[2], sl_vector_t& q_t) {
    if(model_type == omni) {
        double delta_trans, delta_rot, delta_bearing;
        delta_trans = sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2));
        delta_rot = angle_diff(u_t[1].v[2], u_t[0].v[2]);
        delta_bearing = angle_diff(atan2(u_t[1].v[1] - u_t[0].v[1], u_t[1].v[0] - u_t[0].v[0]), u_t[0].v[2]) + q_t.v[2];
        /* Update current robot pose */
        q_t.v[0] += delta_trans*cos(delta_bearing);
        q_t.v[1] += delta_trans*sin(delta_bearing);
        q_t.v[2] = normalize(q_t.v[2] + delta_rot);
    }else if(model_type == diff) {
        double delta_trans, delta_rot1, delta_rot2;
        if(sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2)) < 0.01) {
            delta_rot1 = 0.0;
        }else {
            delta_rot1 = angle_diff(atan2(u_t[1].v[1] - u_t[0].v[1], u_t[1].v[0] - u_t[0].v[0]), u_t[0].v[2]);
        }
        delta_trans = sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2));
        delta_rot2 = angle_diff(angle_diff(u_t[1].v[2], u_t[0].v[2]), delta_rot1);
        /* Update current robot pose */
        q_t.v[0] += delta_trans*cos(q_t.v[2] + delta_rot1);
        q_t.v[1] += delta_trans*sin(q_t.v[2] + delta_rot1);
        q_t.v[2] = normalize(q_t.v[2] + delta_rot1 + delta_rot2);
    }
}

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

double norm2(Eigen::Vector3d& p, Eigen::Vector3d& q) {
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
    cores = setZero(4, num_pointB);
    double d_min;
    int idx;
    for(int i = 0; i < num_pointB; i++) {
        d_min = inf;
        idx = -1;
        for(int j = 0; j < num_pointA; j++) {
            d = norm2(A.col(i), A,col(j));
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
void crossCovariance_Mean(Eigen::MatrixXd& cores, Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::Vector2d& mean_A, Eigen::Vector2d& mean_B, Eigen::Matrix2d& H) {
    mean_A = setZero(2, 1);
    mean_B = setZero(2, 1);
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

    H = setZero(2, 2);
    for(int i = 0; i < num_pairs; i++) {
        int j = cores(1, i);
        if(j != -1) {
            H += (A.col(j) - mean_A)*(B.col(i) - mean_B).transpose()*cores(3, i);
        }
    }
}

/** Vanilla ICP algorithm */
void vanilla_ICP(sl_node_t& node_i, sl_node_t& node_j) {
    Eigen::Matrix2d R_i, R_j, R_ji
    Eigen::Vector2d t_i, t_j, t_ji;
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

    R_ji = R_j.transpose()*R_i;
    t_ji = R_j.transpose()*(t_i - t_j);

    int num_inter = 0;
    Eigen::MatrixXd pcl_ref, pcl_cur;
    pcl_ref = compute_points(node_i);
    pcl_cur = compute_points(node_j);
    Eigen::MatrixXd cores, pcl_temp;
    int num_pointcur = pcl_cur.cols();

    Eigen::Vector2d mean_A;
    Eigen::Vector2d mean_B;
    Eigen::Matrix2d H;
    Eigen::Matrix2d U, V;
    while(num_inter < max_inter) {
        pcl_temp.setZero(2, num_pointcur);
        for(int k = 0; k < num_pointcur; k++) {
            pcl_temp.col(k) = R_ji*pcl_cur(k) + t_ji;
        }
        get_correspondences(cores, pcl_ref, pcl_temp);
        crossCovariance_Mean(cores, pcl_ref, pcl_cur, mean_A, mean_B, H);

        Eigen::JacobiSVD<Matrix2d> svd( H, ComputeThinU | ComputeThinV);
        U = svd.matrixU();
        V = svd.matrixV();
        R_ji = U*V.transpose();
        t_ji = mean_A - R_ji*mean_B;
    }
}