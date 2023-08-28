#pragma once
#include "vk_mcl/vk_mcl.hpp"
#include "vk_mcl/map.hpp"
#include "vk_mcl/parameters.hpp"

double normalize(double z) {
    return atan2(sin(z), cos(z));
}

double angle_diff(double a, double b) {
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

void mean_covariance(pf_set_sample_t& s) {
    /** Compute mean of set samples*/
    double e_x = 0.0, e_y = 0.0, e_theta = 0.0;
    for(int i = 0; i < s.samples.size(); i++) {
        e_x += s.samples[i].weight * s.samples[i].pose.v[0];
        e_y += s.samples[i].weight * s.samples[i].pose.v[1];
        e_theta += s.samples[i].weight * s.samples[i].pose.v[2];
    }
    s.mean.v[0] = e_x; s.mean.v[1] = e_y; s.mean.v[2] = e_theta;

    /** Compute covariance matrix of set samples*/
    double cov_xx = 0.0, cov_yy = 0.0, cov_theta = 0.0;
    for(int i = 0; i < s.samples.size(); i++) {
        cov_xx += s.samples[i].weight * pow(s.samples[i].pose.v[0] - s.mean.v[0], 2);
        cov_yy += s.samples[i].weight * pow(s.samples[i].pose.v[1] - s.mean.v[1], 2);
        cov_theta += s.samples[i].weight * pow(s.samples[i].pose.v[2] - s.mean.v[2], 2);
    }
    s.cov.m[0][0] = cov_xx; s.cov.m[1][1] = cov_yy; s.cov.m[2][2] = cov_theta;
}

/** Uniform sampling in full map*/
void uniform_sample(nav_msgs::OccupancyGrid& map, pf_set_sample_t& s) {
    double x_min = map.info.origin.position.x;
    double x_max = map.info.width*map.info.resolution + map.info.origin.position.x;
    double y_min = map.info.origin.position.y;
    double y_max = map.info.height*map.info.resolution + map.info.origin.position.y;
    random_device seed; mt19937 rng(seed());
	uniform_real_distribution<double> uni_x(x_min, x_max), uni_y(y_min, y_max), uni_theta(-M_PI, M_PI);
    pf_sample_t q;
    s.samples.clear();
    for(int i = 0; i < max_particles; i++) {
        int idx, idy;
        do{
            q.pose.v[0] = uni_x(rng);
            q.pose.v[1] = uni_y(rng);
            q.pose.v[2] = uni_theta(rng);
            idx = (q.pose.v[0] - x_min)/map.info.resolution;
            idy = (q.pose.v[1] - y_min)/map.info.resolution;
        }while(!map_valid(idx, idy, map) || map.data[idy*map.info.width + idx] != free);
        q.weight = 1.0/max_particles;
        s.samples.push_back(q);
    }
    mean_covariance(s);
}

/** Normal sample around initial pose*/
void normal_sample(nav_msgs::OccupancyGrid& map, pf_set_sample_t& s) {
    double x_min = map.info.origin.position.x;
    double x_max = map.info.width*map.info.resolution + map.info.origin.position.x;
    double y_min = map.info.origin.position.y;
    double y_max = map.info.height*map.info.resolution + map.info.origin.position.y;

    double x_0, y_0, theta_0;
    if(!set_init_pose) {
        ifstream last_pose_file;
        last_pose_file.open(last_pose_path);
        if(last_pose_file.is_open()) {
            last_pose_file >> x_0 >> y_0 >> theta_0;
        }else {
            ROS_ERROR("Last pose file can't open!");
            error = true;
        }
        last_pose_file.close();
    }else {
        x_0 = init_pose_x; y_0 = init_pose_y; theta_0 = init_pose_theta;
    }
    random_device seed; mt19937 rng(seed());
    double sigma_x = sqrt(init_cov_x);
    double sigma_y = sqrt(init_cov_y);
    double sigma_theta = sqrt(init_cov_theta);
	normal_distribution<double> nor_x(x_0, sigma_x);
    normal_distribution<double> nor_y(y_0, sigma_y);
    normal_distribution<double> nor_theta(theta_0, sigma_theta);
    pf_sample_t q;
    s.samples.clear();
    for(int i = 0; i < max_particles; i++) {
        int idx, idy;
        do{
            q.pose.v[0] = nor_x(rng);
            q.pose.v[1] = nor_y(rng);
            q.pose.v[2] = nor_theta(rng);
            idx = (q.pose.v[0] - x_min)/map.info.resolution;
            idy = (q.pose.v[1] - y_min)/map.info.resolution;
        }while(!map_valid(idx, idy, map) || map.data[idy*map.info.width + idx] != free);
        q.weight = 1.0/max_particles;
        s.samples.push_back(q);
    }
    mean_covariance(s);
}

/** Motion model*/
pf_vector_t motion_model(pf_vector_t u_t[2], pf_vector_t q_t_1) {
    pf_vector_t q_t;
    switch (model_type) {
        case omni:
            {
                double delta_trans, delta_rot, delta_bearing;
                double delta_trans_hat, delta_rot_hat, delta_strafe_hat;

                delta_trans = sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2));
                delta_rot = angle_diff(u_t[1].v[2], u_t[0].v[2]);

                double sigma_trans, sigma_rot, sigma_strafe;

                sigma_trans = sqrt(anpha3*pow(delta_trans, 2) + anpha1*pow(delta_rot, 2));
                sigma_rot = sqrt(anpha4*pow(delta_rot, 2) + anpha2*pow(delta_trans, 2));
                sigma_strafe = sqrt(anpha1*pow(delta_rot, 2) + anpha5*pow(delta_trans, 2));

                delta_bearing = angle_diff(atan2(u_t[1].v[1] - u_t[0].v[1], u_t[1].v[0] - u_t[0].v[0]), u_t[0].v[2]) + q_t_1.v[2];

                random_device seed; mt19937 rng(seed());
	            normal_distribution<double> nor_trans(0.0, sigma_trans), nor_rot(0.0, sigma_rot), nor_strafe(0.0, sigma_strafe);
                delta_trans_hat = delta_trans + nor_trans(rng);
                delta_rot_hat = delta_rot + nor_rot(rng);
                delta_strafe_hat = 0.0 + nor_strafe(rng);

                q_t.v[0] = q_t_1.v[0] + delta_trans_hat*cos(delta_bearing) + delta_strafe_hat*sin(delta_bearing);
                q_t.v[1] = q_t_1.v[1] + delta_trans_hat*sin(delta_bearing) + delta_strafe_hat*cos(delta_bearing);
                q_t.v[2] = normalize(q_t_1.v[2] + delta_rot_hat);
                break;
            }
        case diff:
            {
                double delta_trans, delta_rot1, delta_rot2;
                double delta_trans_hat, delta_rot1_hat, delta_rot2_hat;
                double delta_rot1_noise, delta_rot2_noise;
                if(sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2)) < 0.01) {
                    delta_rot1 = 0.0;
                }else {
                    delta_rot1 = angle_diff(atan2(u_t[1].v[1] - u_t[0].v[1], u_t[1].v[0] - u_t[0].v[0]), u_t[0].v[2]);
                }
                delta_trans = sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2));
                delta_rot2 = angle_diff(angle_diff(u_t[1].v[2], u_t[0].v[2]), delta_rot1);
                
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

                q_t.v[0] = q_t_1.v[0] + delta_trans_hat*cos(q_t_1.v[2] + delta_rot1_hat);
                q_t.v[1] = q_t_1.v[1] + delta_trans_hat*sin(q_t_1.v[2] + delta_rot1_hat);
                q_t.v[2] = q_t_1.v[2] + delta_rot1_hat + delta_rot2_hat;
                break;
            }
        default:
            {
                ROS_ERROR("Model type of robot incorrect!");
            }
    }
    return q_t;
}

bool update_filter(pf_vector_t u_t[2]) {
    return fabs(sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2))) > min_trans || fabs(angle_diff(u_t[1].v[2], u_t[0].v[2])) > min_rot;
}

void update_motion(pf_vector_t u_t[2], pf_set_sample_t& s) {
    switch (model_type) {
        case omni:
            {
                double delta_trans, delta_rot, delta_bearing;
                delta_trans = sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2));
                delta_rot = angle_diff(u_t[1].v[2], u_t[0].v[2]);
                for(int i = 0; i < s.samples.size(); i++) {
                    delta_bearing = angle_diff(atan2(u_t[1].v[1] - u_t[0].v[1], u_t[1].v[0] - u_t[0].v[0]), u_t[0].v[2]) + s.samples[i].pose.v[2];
                    s.samples[i].pose.v[0] +=  delta_trans*cos(delta_bearing);
                    s.samples[i].pose.v[1] +=  delta_trans*sin(delta_bearing);
                    s.samples[i].pose.v[2] = normalize(s.samples[i].pose.v[2] + delta_rot);
                }
                break;
            }
        case diff:
        {
            double delta_trans, delta_rot1, delta_rot2;
            if(sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2)) < 0.01) {
                delta_rot1 = 0.0;
            }else {
                delta_rot1 = angle_diff(atan2(u_t[1].v[1] - u_t[0].v[1], u_t[1].v[0] - u_t[0].v[0]), u_t[0].v[2]);
            }
            delta_trans = sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2));
            delta_rot2 = angle_diff(angle_diff(u_t[1].v[2], u_t[0].v[2]), delta_rot1);
            for(int i = 0; i < s.samples.size(); i++) {
                s.samples[i].pose.v[0] +=  delta_trans*cos(s.samples[i].pose.v[2] + delta_rot1);
                s.samples[i].pose.v[1] +=  delta_trans*sin(s.samples[i].pose.v[2] + delta_rot1);
                s.samples[i].pose.v[2] = normalize(s.samples[i].pose.v[2] + delta_rot1 + delta_rot2);
            }
            break;
        }
        default:
            {
                ROS_ERROR("Model type of robot incorrect!");
            }
    }
    mean_covariance(s);
}

