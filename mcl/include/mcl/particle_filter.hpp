#pragma once
#include "mcl/mcl.hpp"
#include "mcl/motion_model.hpp"
#include "mcl/sensor_model.hpp"
#include "mcl/map.hpp"

void AugmentedMCL(pf_set_sample_t& S_t_1, pf_vector_t u_t[2], sensor_msgs::LaserScan& scan_t, nav_msgs::OccupancyGrid& map) {
    static double w_slow = 0.0, w_fast = 0.0;
    double w_avg = 0.0, sum_weight = 0.0;
    pf_set_sample_t set_particle_t_;
    set_particle_t_.samples.clear();
    pf_sample_t s;
    scan_handle(S_t_1, scan_t, map);
    for(int i = 0; i < max_particles; i++) {
        s.pose = motion_model(u_t, S_t_1.samples[i].pose);
        if(likelihoodfield) {
            s.weight = likelihoodfield_model(set_scan_t, s.pose, map);
        }else {
            s.weight = beams_model(set_scan_t, s.pose, map);
        }
        set_particle_t_.samples.push_back(s);
        sum_weight += s.weight;
        w_avg = w_avg + s.weight/max_particles;
    }
    double N = 0.0;
    for(int i = 0; i < max_particles; i++) {
        set_particle_t_.samples[i].weight = set_particle_t_.samples[i].weight/sum_weight;
        N += pow(set_particle_t_.samples[i].weight, 2);
    }
    double N_eff = 1.0/N;
    w_slow = w_slow + anpha_slow*(w_avg - w_slow);
    w_fast = w_fast + anpha_fast*(w_avg - w_fast);

    double pro = std::max(0.0, (1.0 - w_fast/w_slow));
    double x_min = map.info.origin.position.x;
    double x_max = map.info.width*occ_map.info.resolution + map.info.origin.position.x;
    double y_min = map.info.origin.position.y;
    double y_max = map.info.height*occ_map.info.resolution + map.info.origin.position.y;

    random_device seed; mt19937 rng(seed());
	uniform_real_distribution<double> uni(0.0,1.0/max_particles), uni_x(x_min, x_max), uni_y(y_min, y_max), uni_theta(-M_PI, M_PI);
	double r = uni(rng);
    double c = set_particle_t_.samples[0].weight;
    int k = 0;
    set_particle_t.samples.clear();
    if(N_eff < (double)max_particles/2) {
        for(int i = 0; i < max_particles; i++) {
            double random = (double)rand()/(RAND_MAX);
            if(random < pro) {
                ROS_INFO("Recover failure!");
                int idx, idy;
                do{
                    s.pose.v[0] = uni_x(rng);
                    s.pose.v[1] = uni_y(rng);
                    s.pose.v[2] = uni_theta(rng);
                    idx = (s.pose.v[0] - x_min)/map.info.resolution;
                    idy = (s.pose.v[1] - y_min)/map.info.resolution;
                }while(!map_valid(idx, idy) || map.data[idy*map.info.width + idx] != free);
                s.weight = 1.0/max_particles;
                set_particle_t.samples.push_back(s);
            }else {
                double U = r + (double)i/max_particles;
                while(U > c) {
                    k = k + 1;
                    c = c + set_particle_t_.samples[k].weight;
                }
                s.pose = set_particle_t_.samples[k].pose;
                s.weight = 1.0/max_particles;
                set_particle_t.samples.push_back(s);
            }
        }
    }else {
        set_particle_t = set_particle_t_;
    }
    compute_mean_covariance(set_particle_t);
}

void KLDSamplingMCL(pf_set_sample_t& S_t_1, pf_vector_t u_t[2], sensor_msgs::LaserScan& scan_t, nav_msgs::OccupancyGrid& map) {
    scan_handle(S_t_1, scan_t, map);
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
    set_particle_t.samples.clear();
    int count = 0;
    do {
        double U;
        U = r + (double)N/max_particles;
        while(U > c) {
            k = k + 1;
            c = c + S_t_1.samples[k].weight;
        }
        pf_vector_t q_t_1 = S_t_1.samples[k].pose;
        s.pose = motion_model(u_t, q_t_1);
        if(likelihoodfield) {
            s.weight = likelihoodfield_model(set_scan_t, s.pose, map);
        }else {
            s.weight = beams_model(set_scan_t, s.pose, map);
        }
        sum_weight += s.weight;
        set_particle_t.samples.push_back(s);

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
    for(int i = 0; i < set_particle_t.samples.size(); i++) {
        set_particle_t.samples[i].weight = set_particle_t.samples[i].weight/sum_weight;
    }
    compute_mean_covariance(set_particle_t);
}

double computeEntropy() {
    double entropy_ = 0.0;
    double sum_weight = 0.0;
    for(int i = 0; i < set_particle_t.samples.size(); i++) {
        sum_weight += set_particle_t.samples[i].weight;
    }
    for(int i = 0; i < set_particle_t.samples.size(); i++) {
        set_particle_t.samples[i].weight = set_particle_t.samples[i].weight/sum_weight;
        entropy_ = entropy_ - set_particle_t.samples[i].weight * log2(set_particle_t.samples[i].weight);
    }
    ROS_INFO("Entropy = %f.", entropy_);
    return entropy_;
}

bool update_filter(pf_vector_t u_t[2]) {
    return fabs(sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2))) > min_trans || fabs(angle_diff(u_t[1].v[2], u_t[0].v[2])) > min_rot;
}

void update_motion(pf_vector_t u_t[2]) {
    if(model_type == omni) {
        double delta_trans, delta_rot, delta_bearing;
        delta_trans = sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2));
        delta_rot = angle_diff(u_t[1].v[2], u_t[0].v[2]);
        for(int i = 0; i < set_particle_t.samples.size(); i++) {
                delta_bearing = angle_diff(atan2(u_t[1].v[1] - u_t[0].v[1], u_t[1].v[0] - u_t[0].v[0]), u_t[0].v[2]) + set_particle_t.samples[i].pose.v[2];
                set_particle_t.samples[i].pose.v[0] +=  delta_trans*cos(delta_bearing);
                set_particle_t.samples[i].pose.v[1] +=  delta_trans*sin(delta_bearing);
                set_particle_t.samples[i].pose.v[2] = normalize(set_particle_t.samples[i].pose.v[2] + delta_rot);
        }
    }else if(model_type == diff) {
        double delta_trans, delta_rot1, delta_rot2;
        if(sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2)) < 0.01) {
            delta_rot1 = 0.0;
        }else {
            delta_rot1 = angle_diff(atan2(u_t[1].v[1] - u_t[0].v[1], u_t[1].v[0] - u_t[0].v[0]), u_t[0].v[2]);
        }
        delta_trans = sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2));
        delta_rot2 = angle_diff(angle_diff(u_t[1].v[2], u_t[0].v[2]), delta_rot1);
        for(int i = 0; i < set_particle_t.samples.size(); i++) {
                set_particle_t.samples[i].pose.v[0] +=  delta_trans*cos(set_particle_t.samples[i].pose.v[2] + delta_rot1);
                set_particle_t.samples[i].pose.v[1] +=  delta_trans*sin(set_particle_t.samples[i].pose.v[2] + delta_rot1);
                set_particle_t.samples[i].pose.v[2] = normalize(set_particle_t.samples[i].pose.v[2] + delta_rot1 + delta_rot2);
        }
    }
}

void mcl_publisher(geometry_msgs::PoseArray& pose_arr, geometry_msgs::PoseWithCovarianceStamped& q_t, sensor_msgs::PointCloud& pcl_, 
    pf_set_sample_t& s, pf_scan_t& scan_t, nav_msgs::OccupancyGrid& map) {
    q_t.header.frame_id = map_frame;
    q_t.pose.pose.position.x = s.mean.v[0];
    q_t.pose.pose.position.y = s.mean.v[1];
    q_t.pose.pose.orientation = tf::createQuaternionMsgFromYaw(s.mean.v[2]);

    q_t.pose.covariance[0] = s.cov.m[0][0];
    q_t.pose.covariance[7] = s.cov.m[1][1];
    q_t.pose.covariance[35] = s.cov.m[2][2];

    if(save_last_pose) {
        ofstream last_pose_file;
        last_pose_file.open(last_pose_path, std::ofstream::out | std::ofstream::trunc);
        if(last_pose_file.is_open()) {
            last_pose_file << q_t.pose.pose.position.x << " " << q_t.pose.pose.position.y << " " << tf::getYaw(q_t.pose.pose.orientation);
        }else {
            ROS_ERROR("Last pose file can't open!");
            error_ = true;
        }
    }

    pf_vector_t pose_lidar;
    pcl_.header.frame_id = map_frame;
    geometry_msgs::Point32 pose_endpoint;
    pcl_.points.clear();
    double theta = tf::getYaw(q_t.pose.pose.orientation);
    pose_lidar.v[0] = q_t.pose.pose.position.x + laser_pose_x*cos(theta) - laser_pose_y*sin(theta);
    pose_lidar.v[1] = q_t.pose.pose.position.y + laser_pose_x*sin(theta) + laser_pose_y*cos(theta);
    int count = 0;
    double d_min, beam_angle;
    float inv_scan;
    if(inverse_laser) {
        inv_scan = -1.0;
    }else {
        inv_scan = 1.0;
    }
    for(int i = 0; i < scan_t.ranges.size(); i++) {
        if(scan_t.ranges[i].v[0] < range_max && scan_t.ranges[i].v[0] >= range_min) {
            beam_angle = inv_scan*scan_t.ranges[i].v[1] + theta + laser_pose_theta;
            pose_endpoint.x = pose_lidar.v[0] + scan_t.ranges[i].v[0]*cos(beam_angle);
            pose_endpoint.y = pose_lidar.v[1] + scan_t.ranges[i].v[0]*sin(beam_angle);
            
            int idx = (pose_endpoint.x - map.info.origin.position.x)/map.info.resolution;
            int idy = (pose_endpoint.y - map.info.origin.position.y)/map.info.resolution;
            int id_cell = idy*map.info.width + idx;
            if(map_valid(idx, idy)) {
                d_min = set_occ_dist[id_cell];
            }else {
                d_min = max_occ_dist;
            }
            if(d_min < err_min) {
                count += 1;
            }
        }
    }
    for(int i = 0; i < laser_scan.ranges.size(); i++) {
        if(laser_scan.ranges[i] < range_max && laser_scan.ranges[i] >= range_min) {
            beam_angle = inv_scan*(i*throttle_scan*angle_increment - angle_max) + theta + laser_pose_theta;
            pose_endpoint.x = pose_lidar.v[0] + laser_scan.ranges[i]*cos(beam_angle);
            pose_endpoint.y = pose_lidar.v[1] + laser_scan.ranges[i]*sin(beam_angle);
            pcl_.points.push_back(pose_endpoint);
        }
    }
    if(count > match_rate*scan_t.ranges.size()) {
        if(!init_localization) {
            ROS_INFO("Matching rate = %.2f", (double)count*100/scan_t.ranges.size());
            ROS_INFO("Initial localization is successfull!");
            init_localization = true;
        }
    }

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