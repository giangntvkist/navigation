#pragma once
#include "vk_mcl/vk_mcl.hpp"
#include "vk_mcl/motion_model.hpp"
#include "vk_mcl/sensor_model.hpp"
#include "vk_mcl/map.hpp"
#include "vk_mcl/parameters.hpp"

/* Augmented Montal Carlo algorithm*/
void AugmentedMCL(pf_set_sample_t& S_t_1, pf_vector_t u_t[2], pf_scan_t& scan, nav_msgs::OccupancyGrid& map, pf_set_sample_t& S_t, 
    pf_scan_t& scan_w, pf_vector_t& laser_pose, vector<double>& map_dist) {
    static double w_slow = 0.0, w_fast = 0.0;
    double w_avg = 0.0, sum_weight = 0.0;
    pf_set_sample_t S_temp;
    S_temp.samples.clear();
    pf_sample_t s;
    scan_filter(S_t_1, scan, map, scan_w, laser_pose, map_dist);
    for(int i = 0; i < max_particles; i++) {
        s.pose = motion_model(u_t, S_t_1.samples[i].pose);
        if(likelihoodfield) {
            s.weight = likelihoodfield_model(scan_w, s.pose, map, laser_pose, map_dist);
        }else {
            s.weight = beams_model(scan_w, s.pose, map, laser_pose);
        }
        S_temp.samples.push_back(s);
        sum_weight += s.weight;
        w_avg = w_avg + s.weight/max_particles;
    }
    double N = 0.0;
    for(int i = 0; i < max_particles; i++) {
        S_temp.samples[i].weight = S_temp.samples[i].weight/sum_weight;
        N += pow(S_temp.samples[i].weight, 2);
    }
    double N_eff = 1.0/N;
    w_slow = w_slow + anpha_slow*(w_avg - w_slow);
    w_fast = w_fast + anpha_fast*(w_avg - w_fast);

    double pro = std::max(0.0, (1.0 - w_fast/w_slow));
    double x_min = map.info.origin.position.x;
    double x_max = map.info.width*map.info.resolution + map.info.origin.position.x;
    double y_min = map.info.origin.position.y;
    double y_max = map.info.height*map.info.resolution + map.info.origin.position.y;

    random_device seed; mt19937 rng(seed());
    double sigma_x = sqrt(init_cov_x);
    double sigma_y = sqrt(init_cov_y);
    double sigma_theta = sqrt(init_cov_theta);
	normal_distribution<double> nor_x(S_t_1.mean.v[0], sigma_x);
    normal_distribution<double> nor_y(S_t_1.mean.v[1], sigma_y);
    normal_distribution<double> nor_theta(S_t_1.mean.v[2], sigma_theta);
    uniform_real_distribution<double> uni(0.0,1.0/max_particles);
	double r = uni(rng);
    double c = S_temp.samples[0].weight;
    int k = 0;
    S_t.samples.clear();
    if(N_eff < 0.5*max_particles) {
        for(int i = 0; i < max_particles; i++) {
            double random = (double)rand()/(RAND_MAX);
            if(random < pro) {
                ROS_INFO("Recover failure!");
                int idx, idy;
                do{
                    s.pose.v[0] = nor_x(rng); s.pose.v[1] = nor_y(rng); s.pose.v[2] = nor_theta(rng);
                    idx = (s.pose.v[0] - x_min)/map.info.resolution;
                    idy = (s.pose.v[1] - y_min)/map.info.resolution;
                }while(!map_valid(idx, idy, map) || map.data[idy*map.info.width + idx] != free);
                s.weight = 1.0/max_particles;
                S_t.samples.push_back(s);
            }else {
                double U = r + (double)i/max_particles;
                while(U > c) {
                    k = k + 1;
                    c = c + S_temp.samples[k].weight;
                }
                s.pose = S_temp.samples[k].pose;
                s.weight = 1.0/max_particles;
                S_t.samples.push_back(s);
            }
        }
    }else {
        for(int i = 0; i < max_particles; i++) {
            S_t.samples.push_back(S_temp.samples[i]);
        }
    }
    mean_covariance(S_t);
}

/* KLD Montal Carlo algorithm*/
void KLDSamplingMCL(pf_set_sample_t& S_t_1, pf_vector_t u_t[2], pf_scan_t& scan, nav_msgs::OccupancyGrid& map, pf_set_sample_t& S_t, 
    pf_scan_t& scan_w, pf_vector_t& laser_pose, vector<double>& map_dist) {
    scan_filter(S_t_1, scan, map, scan_w, laser_pose, map_dist);
    pf_sample_t s;
    double x_min = map.info.origin.position.x;
    double x_max = map.info.width*map.info.resolution + map.info.origin.position.x;
    double y_min = map.info.origin.position.y;
    double y_max = map.info.height*map.info.resolution + map.info.origin.position.y;

    vector<int> H;
    H.clear();
    int width = (x_max - x_min)/bin_size_x + 1;
    int lenght = (y_max - y_min)/bin_size_y + 1;
    int height = 2*M_PI/bin_size_theta + 1;
    for(int i = 0; i < height; i++) {
        for(int j = 0; j < lenght; j++) {
            for(int k = 0; k < width; k++) {
                H.push_back(empt);
            }
        }
    }

    random_device seed; mt19937 rng(seed());
	uniform_real_distribution<double> uni(0.0,1.0/max_particles);
	double r = uni(rng);
    double c = S_t_1.samples[0].weight;
    int N = 0, N_s = 0, n = 0, k = 0;
    double sum_weight = 0;
    S_t.samples.clear();
    int count = 0;
    do {
        double U;
        U = r + (double)N/max_particles;
        while(U > c) {
            k = k + 1;
            c = c + S_t_1.samples[k].weight;
        }
        s.pose = motion_model(u_t, S_t_1.samples[k].pose);
        if(likelihoodfield) {
            s.weight = likelihoodfield_model(scan_w, s.pose, map, laser_pose, map_dist);
        }else {
            s.weight = beams_model(scan_w, s.pose, map, laser_pose);
        }
        sum_weight += s.weight;
        S_t.samples.push_back(s);

        int id_x = (s.pose.v[0] - x_min)/bin_size_x;
        int id_y = (s.pose.v[1] - y_min)/bin_size_y;
        int id_theta = (s.pose.v[2] + M_PI)/bin_size_theta;
        int id_bin = id_theta*width*lenght + id_y*width +id_x;
        if(id_x >= 0 && id_x < width && id_y >=0 && id_y < lenght && id_bin < H.size()) {
            if(H[id_bin] == empt) {
                n = n + 1;
                H[id_bin] = no_empt;
                if(n > 1) {
                    double a = (n-1)/(2*kld_eps);
                    double b = 2/(9*(n-1));
                    N_s = a*pow((1 - b + sqrt(b)*kld_delta), 3);
                }
            }
        }else {
            ROS_WARN("Bin size invalid!");
        }
        N = N + 1;
        if(N == max_particles) {
            break;
        }
    }while (N < N_s || N < min_particles);
    for(int i = 0; i < S_t.samples.size(); i++) {
        S_t.samples[i].weight = S_t.samples[i].weight/sum_weight;
    }
    mean_covariance(S_t);
}

/** Compute entropy */
double computeEntropy(pf_set_sample_t& S_t) {
    double entropy_ = 0.0;
    double sum_weight = 0.0;
    for(int i = 0; i < S_t.samples.size(); i++) {
        sum_weight += S_t.samples[i].weight;
    }
    for(int i = 0; i < S_t.samples.size(); i++) {
        S_t.samples[i].weight = S_t.samples[i].weight/sum_weight;
        entropy_ = entropy_ - S_t.samples[i].weight * log2(S_t.samples[i].weight);
    }
    ROS_INFO("Entropy = %f.", entropy_);
    return entropy_;
}

/** Publisher robot pose and point cloud laser*/
void mcl_publisher(geometry_msgs::PoseArray& pose_arr, geometry_msgs::PoseWithCovarianceStamped& q_t, sensor_msgs::PointCloud& pcl, 
    pf_set_sample_t& s, pf_vector_t laser_pose, pf_scan_t& scan, nav_msgs::OccupancyGrid& map) {
    /* Robot pose publisher */
    q_t.header.frame_id = map_frame;
    q_t.pose.pose.position.x = s.mean.v[0];
    q_t.pose.pose.position.y = s.mean.v[1];
    q_t.pose.pose.orientation = tf::createQuaternionMsgFromYaw(s.mean.v[2]);

    q_t.pose.covariance[0] = s.cov.m[0][0];
    q_t.pose.covariance[7] = s.cov.m[1][1];
    q_t.pose.covariance[35] = s.cov.m[2][2];

    /* Save last pose*/
    if(save_last_pose) {
        ofstream last_pose_file;
        last_pose_file.open(last_pose_path, std::ofstream::out | std::ofstream::trunc);
        if(last_pose_file.is_open()) {
            ROS_INFO("Saving last pose!");
            last_pose_file << q_t.pose.pose.position.x << " " << q_t.pose.pose.position.y << " " << tf::getYaw(q_t.pose.pose.orientation);
            ROS_INFO("Saved!");
        }else {
            ROS_ERROR("Last pose file can't open!");
            error = true;
        }
    }

    /* Point cloud publisher */
    pf_vector_t pose_lidar;
    geometry_msgs::Point32 pose_endpoint;
    pcl.header.frame_id = map_frame;
    pcl.points.clear();
    double theta = tf::getYaw(q_t.pose.pose.orientation);
    pose_lidar.v[0] = q_t.pose.pose.position.x + laser_pose.v[0]*cos(theta) - laser_pose.v[1]*sin(theta);
    pose_lidar.v[1] = q_t.pose.pose.position.y + laser_pose.v[0]*sin(theta) + laser_pose.v[1]*cos(theta);
    double beam_angle;
    for(int i = 0; i < scan.ranges.size(); i++) {
        if(scan.ranges[i].v[0] < range_max && scan.ranges[i].v[0] >= range_min) {
            beam_angle = scan.ranges[i].v[1] + theta + laser_pose.v[2];
            pose_endpoint.x = pose_lidar.v[0] + scan.ranges[i].v[0]*cos(beam_angle);
            pose_endpoint.y = pose_lidar.v[1] + scan.ranges[i].v[0]*sin(beam_angle);
            pcl.points.push_back(pose_endpoint);
        }
    }

    /* Pose array publisher */
    pose_arr.header.frame_id = map_frame;
    pose_arr.poses.clear();
    for(int i = 0; i < s.samples.size(); i++) {
        geometry_msgs::Pose p;
        p.position.x = s.samples[i].pose.v[0];
        p.position.y = s.samples[i].pose.v[1];
        p.position.z = 0;
        p.orientation = tf::createQuaternionMsgFromYaw(s.samples[i].pose.v[2]);
        pose_arr.poses.push_back(p);
    }
}

