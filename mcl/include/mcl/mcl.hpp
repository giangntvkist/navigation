#pragma once
#include "mcl/header.hpp"

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    occ_map.info.width = msg->info.width;
    occ_map.info.height = msg->info.height;
    occ_map.info.resolution = msg->info.resolution;
    occ_map.info.origin.position.x = msg->info.origin.position.x;
    occ_map.info.origin.position.y = msg->info.origin.position.y;
    occ_map.data.clear();
    for(int i = 0; i < msg->data.size(); i++) {
        occ_map.data.push_back(msg->data[i]);
    }
    _map = true;
}
void dataCallback(const nav_msgs::Odometry& msg, const sensor_msgs::LaserScan& scan) {
    laser_scan.angle_min = scan.angle_min;
    laser_scan.angle_max = scan.angle_max;
    laser_scan.angle_increment = scan.angle_increment;
    laser_scan.range_min = scan.range_min;
    laser_scan.range_max = range_max;
    laser_scan.position.x = laser_pose_x;
    laser_scan.position.y = laser_pose_y;
    laser_scan.position.theta = laser_pose_theta;
    laser_scan.ranges.clear();
    for(int i = 0; i < scan.ranges.size(); i++) {
        laser_scan.ranges.push_back(scan.ranges[i]);
    }
    odom.pose.pose.position.x = msg.pose.pose.position.x;
	odom.pose.pose.position.y = msg.pose.pose.position.y;
	odom.pose.pose.orientation = msg.pose.pose.orientation;
    _data = true;
}
bool map_valid(int idx, int idy) {
    return (idx >=0 && idx < occ_map.info.width && idy >= 0 && idy < occ_map.info.height);
}
void get_lookuptable() {
    if(_map) {
        ROS_INFO("Waiting for get the lookup table...");
        if(lookup_table) {
            int idx, idy;
            double w;
            double a = z_hit/sqrt(2*M_PI*pow(sigma_hit, 2));
            double b_min = -0.5*pow(dist_max/sigma_hit, 2);
            double w_min = a*exp(b_min) + z_rand/range_max;
            double w_max = a + z_rand/range_max;
            int kernel_size = dist_max/occ_map.info.resolution;
            set_weight.clear();
            set_weight.resize(occ_map.data.size());
            for(int i = 0; i < occ_map.data.size(); i++) {
                cout << "\r";
                cout << "Loading the lookup table: " << int((i+1)*100/occ_map.data.size()) << "% ...";
                if(occ_map.data[i] != occupied) {
                    set_weight[i] = std::max(w_min, set_weight[i]);
                }else {
                    idy = i/occ_map.info.width;
                    idx = i - idy*occ_map.info.width;
                    for(int j = idy - kernel_size; j <= idy + kernel_size; j++) {
                        if(j >= 0 && j < occ_map.info.height) {
                            for(int k = idx - kernel_size; k <= idx + kernel_size; k++) {
                                if(k >= 0 && k < occ_map.info.width) {
                                    if(occ_map.data[j*occ_map.info.width + k] != occupied) {
                                        double d = occ_map.info.resolution*sqrt(pow(idx - k, 2) + pow(idy - j, 2));
                                        double b = -0.5*pow(d/sigma_hit, 2);
                                        w = a*exp(b) + z_rand/range_max;
                                    }else {
                                        w = w_max;
                                    }
                                    set_weight[j*occ_map.info.width + k] = std::max(w, set_weight[j*occ_map.info.width + k]);
                                }
                            }
                        }
                    }
                }
            }
            ofstream lookup_table_file(lookup_table_path, std::ofstream::out | std::ofstream::trunc);
            if(lookup_table_file.fail()) {
                ROS_ERROR("Invalid lookup table link!");
                error_ = true;
            }else {
                for(int i = 0; i < occ_map.info.height; i++) {
                    for(int j = 0; j < occ_map.info.width; j++) {
                        lookup_table_file << set_weight[i*occ_map.info.width+j] << " ";
                    }
                    lookup_table_file << endl;
                }
            }
            lookup_table_file.close();
            ROS_INFO("Got the lookup table!");
        }else {
            ifstream lookup_table_file;
            lookup_table_file.open(lookup_table_path);
            set_weight.clear();
            double w;
            if(lookup_table_file.fail()) {
                ROS_ERROR("Invalid lookup table link!");
                error_ = true;
            }else {
                while(!lookup_table_file.eof()) {
                    if(lookup_table_file >> w) {
                        set_weight.push_back(w);
                    }else {
                        break;
                    }
                }
                lookup_table_file.close();
                if(set_weight.size() != occ_map.data.size()) {
                    ROS_ERROR("Size of lookup table is not equal to size map!");
                    error_ = true;
                }else {
                    ROS_INFO("Got the lookup table!");
                }
            }   
        }
        first_time = false;
    }
}
void uniform_sample(nav_msgs::OccupancyGrid& map) {
    double x_min = map.info.origin.position.x;
    double x_max = map.info.width*map.info.resolution + map.info.origin.position.x;
    double y_min = map.info.origin.position.y;
    double y_max = map.info.height*map.info.resolution + map.info.origin.position.y;
    random_device seed;
	mt19937 rng(seed());
	uniform_real_distribution<double> uni_x(x_min,x_max), uni_y(y_min,y_max), uni_theta(-M_PI, M_PI);
    Particle q;
    set_particle_t.clear();
    for(int i = 0; i < max_particles; i++) {
        int idx, idy;
        do{
            q.pose.x = uni_x(rng);
            q.pose.y = uni_y(rng);
            q.pose.theta = uni_theta(rng);
            idx = (q.pose.x - x_min)/map.info.resolution;
            idy = (q.pose.y - y_min)/map.info.resolution;
        }while(!map_valid(idx, idy) || map.data[idy*map.info.width + idx] != free);
        q.weight = 1.0/max_particles;
        set_particle_t.push_back(q);
    }
}
void normal_sample() {
    double x_min = occ_map.info.origin.position.x;
    double x_max = occ_map.info.width*occ_map.info.resolution + occ_map.info.origin.position.x;
    double y_min = occ_map.info.origin.position.y;
    double y_max = occ_map.info.height*occ_map.info.resolution + occ_map.info.origin.position.y;

    double x_0, y_0, theta_0;
    if(!set_init_pose) {
        ifstream last_pose_file;
        last_pose_file.open(last_pose_path);
        if(last_pose_file.fail()) {
            ROS_ERROR("Invalid last pose link!");
            error_ = true;
        }else {
            last_pose_file >> x_0 >> y_0 >> theta_0;
        }
        last_pose_file.close();
    }else {
        x_0 = init_pose_x;
        y_0 = init_pose_y;
        theta_0 = init_pose_theta;
    }
    random_device seed;
	mt19937 rng(seed());
	normal_distribution<double> nor_x(x_0, 0.5*init_cov_x);
    normal_distribution<double> nor_y(y_0, 0.5*init_cov_y);
    normal_distribution<double> nor_theta(theta_0, 0.5*init_cov_theta);
    
    Particle q;
    set_particle_t.clear();
    for(int i = 0; i < max_particles; i++) {
        int idx, idy;
        do{
            q.pose.x = nor_x(rng);
            q.pose.y = nor_y(rng);
            q.pose.theta = nor_theta(rng);
            idx = (q.pose.x - x_min)/occ_map.info.resolution;
            idy = (q.pose.y - y_min)/occ_map.info.resolution;
        }while(!map_valid(idx, idy) || occ_map.data[idy*occ_map.info.width + idx] != free);
        q.weight = 1.0/max_particles;
        set_particle_t.push_back(q);
    }
}
inline double normalize(double z) {
    return atan2(sin(z), cos(z));
}
inline double angle_diff(double a, double b) {
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

Pose motion_model(Pose u_t[2], Pose q_t_1) {
    Pose q_t;
    switch (model_type) {
        case omni:
            {
                double delta_trans, delta_rot, delta_bearing;
                double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

                delta_trans = sqrt(pow(u_t[1].x - u_t[0].x, 2) + pow(u_t[1].y - u_t[0].y, 2));
                delta_rot = angle_diff(u_t[1].theta, u_t[0].theta);

                double sigma_trans, sigma_rot, sigma_strafe;

                sigma_trans = sqrt(anpha3*pow(delta_trans, 2) + anpha1*pow(delta_rot, 2));
                sigma_rot = sqrt(anpha4*pow(delta_rot, 2) + anpha2*pow(delta_trans, 2));
                sigma_strafe = sqrt(anpha1*pow(delta_rot, 2) + anpha5*pow(delta_trans, 2));

                delta_bearing = angle_diff(atan2(u_t[1].y - u_t[0].y, u_t[1].x - u_t[0].x), u_t[0].theta) + q_t_1.theta;
                random_device seed;
	            mt19937 rng(seed());
	            normal_distribution<double> nor_trans(0.0, sigma_trans), nor_rot(0.0, sigma_rot), nor_strafe(0.0, sigma_strafe);
                delta_trans_hat = delta_trans + nor_trans(rng);
                delta_rot_hat = delta_rot + nor_rot(rng);
                delta_strafe_hat = 0.0 + nor_strafe(rng);

                q_t.x = q_t_1.x + delta_trans_hat*cos(delta_bearing) + delta_strafe_hat*sin(delta_bearing);
                q_t.y = q_t_1.y + delta_trans_hat*sin(delta_bearing) + delta_strafe_hat*cos(delta_bearing);
                q_t.theta = normalize(q_t_1.theta + delta_rot_hat);
                break;
            }
        case diff:
            {
                double delta_trans, delta_rot1, delta_rot2;
                double delta_trans_hat, delta_rot1_hat, delta_rot2_hat;
                double delta_rot1_noise, delta_rot2_noise;
                if(sqrt(pow(u_t[1].x - u_t[0].x, 2) + pow(u_t[1].y - u_t[0].y, 2)) < 0.01) {
                    delta_rot1 = 0.0;
                }else {
                    delta_rot1 = angle_diff(atan2(u_t[1].y - u_t[0].y, u_t[1].x - u_t[0].x), u_t[0].theta);
                }
                delta_trans = sqrt(pow(u_t[1].x - u_t[0].x, 2) + pow(u_t[1].y - u_t[0].y, 2));
                delta_rot2 = angle_diff(angle_diff(u_t[1].theta, u_t[0].theta), delta_rot1);
                
                delta_rot1_noise = std::min(fabs(angle_diff(delta_rot1, 0.0)), fabs(angle_diff(delta_rot1, M_PI)));
                delta_rot2_noise = std::min(fabs(angle_diff(delta_rot2, 0.0)), fabs(angle_diff(delta_rot2, M_PI)));

                double sigma_trans, sigma_rot1, sigma_rot2;
                sigma_rot1 = sqrt(anpha1*pow(delta_rot1_noise, 2) + anpha2*pow(delta_trans, 2));
	            sigma_trans = sqrt(anpha3*pow(delta_trans, 2) + anpha4*pow(delta_rot1_noise, 2) + anpha4*pow(delta_rot2_noise, 2));
	            sigma_rot2 = sqrt(anpha1*pow(delta_rot2_noise, 2) + anpha2*pow(delta_trans, 2));
                

                random_device seed;
	            mt19937 rng(seed());
	            normal_distribution<double> nor_rot1(0.0, sigma_rot1), nor_trans(0.0, sigma_trans), nor_rot2(0.0, sigma_rot2);

                delta_rot1_hat = angle_diff(delta_rot1, nor_rot1(rng));
	            delta_trans_hat = delta_trans - nor_trans(rng);
	            delta_rot2_hat = angle_diff(delta_rot2, nor_rot2(rng));

                q_t.x = q_t_1.x + delta_trans_hat*cos(q_t_1.theta + delta_rot1_hat);
                q_t.y = q_t_1.y + delta_trans_hat*sin(q_t_1.theta + delta_rot1_hat);
                q_t.theta = q_t_1.theta + delta_rot1_hat + delta_rot2_hat;
                break;
            }
        default:
            {
                ROS_ERROR("Model type of robot incorrect!");
            }
    }
    return q_t;
}
double likelihoodfield_model(LaserData& scan_t, Pose q_t, nav_msgs::OccupancyGrid& map) {
    double q = 1;
    double a = z_hit/sqrt(2*M_PI*pow(sigma_hit, 2));
    double b = -0.5*pow(dist_max/sigma_hit, 2);
    Pose pose_lidar, pose_endpoint;
    pose_lidar.x = q_t.x + scan_t.position.x*cos(q_t.theta) - scan_t.position.y*sin(q_t.theta);
    pose_lidar.y = q_t.y + scan_t.position.x*sin(q_t.theta) + scan_t.position.y*cos(q_t.theta);
    float inv_scan;
    if(inverse_laser) {
        inv_scan = -1.0;
    }else {
        inv_scan = 1.0;
    }
    for(int i = 0; i < scan_t.ranges.size(); i += throttle_scan) {
        if(scan_t.ranges[i] != scan_t.range_max) {
            pose_endpoint.x = pose_lidar.x + scan_t.ranges[i]*cos(inv_scan*(i*scan_t.angle_increment - scan_t.angle_max) + q_t.theta + scan_t.position.theta);
            pose_endpoint.y = pose_lidar.y + scan_t.ranges[i]*sin(inv_scan*(i*scan_t.angle_increment - scan_t.angle_max) + q_t.theta + scan_t.position.theta);
            
            int idx = (pose_endpoint.x - map.info.origin.position.x)/map.info.resolution;
            int idy = (pose_endpoint.y - map.info.origin.position.y)/map.info.resolution;
            int id_cell = idy*map.info.width + idx;
            if(map_valid(idx, idy)) {
                q *= set_weight[id_cell];
            }else {
                q *= a*exp(b) + z_rand/range_max;
            }
        }
    }
    return q;
}
void AugmentedMCL(vector<Particle>& S_t_1, Pose u_t[2], LaserData& scan_t, nav_msgs::OccupancyGrid& map) {
    static double w_slow = 0.0, w_fast = 0.0;
    double w_avg = 0.0, sum_weight = 0.0;

    vector<Particle> set_particle_t_;
    set_particle_t_.clear();
    Particle s;

    for(int i = 0; i < max_particles; i++) {
        s.pose = motion_model(u_t, S_t_1[i].pose);
        s.weight = likelihoodfield_model(scan_t, s.pose, map);
        set_particle_t_.push_back(s);

        sum_weight += s.weight;
        w_avg = w_avg + s.weight/max_particles;
    }
    double N = 0.0;
    for(int i = 0; i < max_particles; i++) {
        set_particle_t_[i].weight = set_particle_t_[i].weight/sum_weight;
        N += pow(set_particle_t_[i].weight, 2);
    }
    double N_eff = 1.0/N;
    
    w_slow = w_slow + anpha_slow*(w_avg - w_slow);
    w_fast = w_fast + anpha_fast*(w_avg - w_fast);

    double pro = std::max(0.0, (1.0 - w_fast/w_slow));

    double x_min = map.info.origin.position.x;
    double x_max = map.info.width*occ_map.info.resolution + map.info.origin.position.x;
    double y_min = map.info.origin.position.y;
    double y_max = map.info.height*occ_map.info.resolution + map.info.origin.position.y;

    random_device seed;
    mt19937 rng(seed());
	uniform_real_distribution<double> uni(0.0,1.0/max_particles), uni_x(x_min,x_max), uni_y(y_min,y_max), uni_theta(-M_PI, M_PI);
	double r = uni(rng);
    double c = set_particle_t_[0].weight;
    int k = 0;
    set_particle_t.clear();
    if(N_eff < (double)max_particles/2) {
        for(int i = 0; i < max_particles; i++) {
            double random = (double)rand()/(RAND_MAX);
            if(random < pro) {
                ROS_INFO("Recover failure!");
                int idx, idy;
                do{
                    s.pose.x = uni_x(rng);
                    s.pose.y = uni_y(rng);
                    s.pose.theta = uni_theta(rng);
                    idx = (s.pose.x - x_min)/map.info.resolution;
                    idy = (s.pose.y - y_min)/map.info.resolution;
                }while(!map_valid(idx, idy) || map.data[idy*map.info.width + idx] != free);
                s.weight = 1.0/max_particles;
                set_particle_t.push_back(s);
            }else {
                double U = r + (double)i/max_particles;
                while(U > c) {
                    k = k + 1;
                    c = c + set_particle_t_[k].weight;
                }
                s.pose = set_particle_t_[k].pose;
                s.weight = 1.0/max_particles;
                set_particle_t.push_back(s);
            }
        }
    }else {
        set_particle_t = set_particle_t_;
    }
}
void KLDSamplingMCL(vector<Particle>& S_t_1, Pose u_t[2], LaserData& scan_t, nav_msgs::OccupancyGrid& map) {
    Particle s;
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

    random_device seed;
    mt19937 rng(seed());
	uniform_real_distribution<double> uni(0.0,1.0/max_particles);
	double r = uni(rng);
    double c = S_t_1[0].weight;
    int N = 0, N_s = 0, n = 0, k = 0;
    double sum_weight = 0;
    set_particle_t.clear();
    int count = 0;
    do {
        double U;
        U = r + (double)N/max_particles;
        while(U > c) {
            k = k + 1;
            c = c + S_t_1[k].weight;
        }
        Pose q_t_1 = S_t_1[k].pose;
        s.pose = motion_model(u_t, q_t_1);
        s.weight = likelihoodfield_model(scan_t, s.pose, map);
        sum_weight += s.weight;
        set_particle_t.push_back(s);

        int id_x = (s.pose.x - x_min)/bin_size_x;
        int id_y = (s.pose.y - y_min)/bin_size_y;
        int id_theta = (s.pose.theta + M_PI)/bin_size_theta;
        int id_bin = id_theta*width*lenght + id_y*width +id_x;
        if(id_bin < H.size() && id_x >= 0 && id_x < width && id_y >=0 && id_y < lenght) {
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
    for(int i = 0; i < set_particle_t.size(); i++) {
        set_particle_t[i].weight = set_particle_t[i].weight/sum_weight;
    }
}
geometry_msgs::PoseWithCovarianceStamped MeanAndCovariance() {
    geometry_msgs::PoseWithCovarianceStamped q;
    q.header.frame_id = map_frame;
    double mean_x = 0.0, mean_y = 0.0, mean_theta = 0.0;
    for(int i = 0; i < set_particle_t.size(); i++) {
        mean_x += set_particle_t[i].weight*set_particle_t[i].pose.x;
        mean_y += set_particle_t[i].weight*set_particle_t[i].pose.y;
        mean_theta += set_particle_t[i].weight*set_particle_t[i].pose.theta;
    }
    q.pose.pose.position.x = mean_x;
    q.pose.pose.position.y = mean_y;
    q.pose.pose.orientation = tf::createQuaternionMsgFromYaw(mean_theta);

    double cov_x = 0.0, cov_y = 0.0, cov_theta = 0.0;
    mean_theta = tf::getYaw(q.pose.pose.orientation);
    for(int i = 0; i < set_particle_t.size(); i++) {
        cov_x += set_particle_t[i].weight*pow(set_particle_t[i].pose.x - q.pose.pose.position.x, 2);
        cov_y += set_particle_t[i].weight*pow(set_particle_t[i].pose.y - q.pose.pose.position.y, 2);
        cov_theta += set_particle_t[i].weight*pow(set_particle_t[i].pose.theta - mean_theta, 2);
    }
    q.pose.covariance[0] = mean_x;
    q.pose.covariance[7] = mean_y;
    q.pose.covariance[35] = mean_theta;
    if(save_last_pose) {
        ofstream last_pose_file;
        last_pose_file.open(last_pose_path, std::ofstream::out | std::ofstream::trunc);
        if(last_pose_file.fail()) {
            ROS_ERROR("Invalid last pose link!");
            error_ = true;
        }else {
            last_pose_file << q.pose.pose.position.x << " " << q.pose.pose.position.y << " " << mean_theta;
        }
        last_pose_file.close();
    }
    return q;
}
double computeEntropy() {
    double entropy = 0.0;
    double sum_weight = 0.0;
    for(int i = 0; i < set_particle_t.size(); i++) {
        sum_weight += set_particle_t[i].weight;
    }
    for(int i = 0; i < set_particle_t.size(); i++) {
        set_particle_t[i].weight = set_particle_t[i].weight/sum_weight;
        entropy = entropy - set_particle_t[i].weight * log2(set_particle_t[i].weight);
    }
    ROS_INFO("Entropy = %f.", entropy);
    return entropy;
}
sensor_msgs::PointCloud PointCloudExport(LaserData& scan_t, geometry_msgs::PoseWithCovarianceStamped q_t, nav_msgs::OccupancyGrid& map) {
    sensor_msgs::PointCloud PCL;
    PCL.header.frame_id = map_frame;
    Pose pose_lidar;
    geometry_msgs::Point32 pose_endpoint;
    double theta = tf::getYaw(q_t.pose.pose.orientation);
    pose_lidar.x = q_t.pose.pose.position.x + scan_t.position.x*cos(theta) - scan_t.position.y*sin(theta);
    pose_lidar.y = q_t.pose.pose.position.y + scan_t.position.x*sin(theta) + scan_t.position.y*cos(theta);
    int count = 0;
    double w;
    double a = z_hit/sqrt(2*M_PI*pow(sigma_hit, 2));
    double b = -0.5*pow(dist_max/sigma_hit, 2);
    double b_min = -0.5*pow(err_min/sigma_hit, 2);

    float inv_scan;
    if(inverse_laser) {
        inv_scan = -1.0;
    }else {
        inv_scan = 1.0;
    }
    double w_b = a*exp(b) + z_rand/range_max;
    double w_b_min = a*exp(b_min) + z_rand/range_max;
    for(int i = 0; i < scan_t.ranges.size(); i += outlier_point) {
        pose_endpoint.x = pose_lidar.x + scan_t.ranges[i]*cos(inv_scan*(i*scan_t.angle_increment - scan_t.angle_max) + theta + scan_t.position.theta);
        pose_endpoint.y = pose_lidar.y + scan_t.ranges[i]*sin(inv_scan*(i*scan_t.angle_increment - scan_t.angle_max) + theta + scan_t.position.theta);

        int idx = (pose_endpoint.x - map.info.origin.position.x)/map.info.resolution;
        int idy = (pose_endpoint.y - map.info.origin.position.y)/map.info.resolution;
        int id_cell = idy*map.info.width + idx;
        if(map_valid(idx, idy)) {
            w = set_weight[id_cell];
        }else {
            w = w_b;
        }

        if(w > w_b_min) {
            count += 1;
        }
        PCL.points.push_back(pose_endpoint);
    }
    int num_points = scan_t.ranges.size()/outlier_point;
    if(count > match_rate*num_points) {
        if(!init_localization) {
            ROS_INFO("Matching rate = %.2f", (double)count*100/num_points);
            ROS_INFO("Initial localization is successfull!");
            init_localization = true;
        }
    }
    return PCL;
}
bool update_filter(Pose u_t[2]) {
    return sqrt(pow(u_t[1].x - u_t[0].x, 2) + pow(u_t[1].y - u_t[0].y, 2)) > min_trans || angle_diff(u_t[1].theta, u_t[0].theta) > min_rot;
}
void update_motion(Pose u_t[2]) {
    if(model_type == omni) {
        double delta_trans, delta_rot, delta_bearing;
        delta_trans = sqrt(pow(u_t[1].x - u_t[0].x, 2) + pow(u_t[1].y - u_t[0].y, 2));
        delta_rot = angle_diff(u_t[1].theta, u_t[0].theta);
        for(int i = 0; i < set_particle_t.size(); i++) {
                delta_bearing = angle_diff(atan2(u_t[1].y - u_t[0].y, u_t[1].x - u_t[0].x), u_t[0].theta) + set_particle_t[i].pose.theta;
                set_particle_t[i].pose.x +=  delta_trans*cos(delta_bearing);
                set_particle_t[i].pose.y +=  delta_trans*sin(delta_bearing);
                set_particle_t[i].pose.theta = normalize(set_particle_t[i].pose.theta + delta_rot);
        }
    }else if(model_type == diff) {
        double delta_trans, delta_rot1, delta_rot2;
        if(sqrt(pow(u_t[1].x - u_t[0].x, 2) + pow(u_t[1].y - u_t[0].y, 2)) < 0.01) {
            delta_rot1 = 0.0;
        }else {
            delta_rot1 = angle_diff(atan2(u_t[1].y - u_t[0].y, u_t[1].x - u_t[0].x), u_t[0].theta);
        }
        delta_trans = sqrt(pow(u_t[1].x - u_t[0].x, 2) + pow(u_t[1].y - u_t[0].y, 2));
        delta_rot2 = angle_diff(angle_diff(u_t[1].theta, u_t[0].theta), delta_rot1);
        for(int i = 0; i < set_particle_t.size(); i++) {
                set_particle_t[i].pose.x +=  delta_trans*cos(set_particle_t[i].pose.theta + delta_rot1);
                set_particle_t[i].pose.y +=  delta_trans*sin(set_particle_t[i].pose.theta + delta_rot1);
                set_particle_t[i].pose.theta = normalize(set_particle_t[i].pose.theta + delta_rot1 + delta_rot2);
        }
    }
}
