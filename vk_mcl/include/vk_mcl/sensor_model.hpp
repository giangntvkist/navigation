#pragma once
#include "vk_mcl/vk_mcl.hpp"
#include "vk_mcl/map.hpp"
#include "vk_mcl/parameters.hpp"

void scan_filter(pf_set_sample_t& s, pf_scan_t& scan, nav_msgs::OccupancyGrid& map, pf_scan_t& scan_w, pf_vector_t& laser_pose, vector<double>& map_dist) {
    bool converged = true;
    for(int i = 0; i < s.samples.size(); i++) {
        if(sqrt(pow(s.samples[i].pose.v[0] - s.mean.v[0], 2) + pow(s.samples[i].pose.v[1] - s.mean.v[1], 2)) > converged_distance) {
            converged = false;
            break;
        }
    }
    pf_vector_t pose_lidar, pose_endpoint, r;
    int idx, idy, id_cell;
    double beam_angle;
    if(converged) {
        int count_scan = 0;
        scan_w.ranges.clear();
        for(int i = 0; i < scan.ranges.size(); i++) {
            int count_particles = 0;
            for(int j = 0; j < s.samples.size(); j++) {
                pose_lidar.v[0] = s.samples[j].pose.v[0] + laser_pose.v[0]*cos(s.samples[j].pose.v[2]) - laser_pose.v[1]*sin(s.samples[j].pose.v[2]);
                pose_lidar.v[1] = s.samples[j].pose.v[1] + laser_pose.v[0]*sin(s.samples[j].pose.v[2]) + laser_pose.v[1]*cos(s.samples[j].pose.v[2]);
                beam_angle = scan.ranges[i].v[1] + s.samples[j].pose.v[2] + laser_pose.v[2];

                pose_endpoint.v[0] = pose_lidar.v[0] + scan.ranges[i].v[0]*cos(beam_angle);
                pose_endpoint.v[1] = pose_lidar.v[1] + scan.ranges[i].v[0]*sin(beam_angle);
                idx = (pose_endpoint.v[0] - map.info.origin.position.x)/map.info.resolution;
                idy = (pose_endpoint.v[1] - map.info.origin.position.y)/map.info.resolution;
                id_cell = idy*map.info.width + idx;
                if(!map_valid(idx, idy, map) || map_dist[id_cell] > beam_skip_distance) {
                    count_particles += 1;
                }
            }
            if(count_particles > beam_skip_threshold*s.samples.size()) {
                count_scan += 1;
            }else {
                r = scan.ranges[i];
                scan_w.ranges.push_back(r);
            }
        }
        if(count_scan > beam_skip_error_threshold*scan.ranges.size()) {
            scan_w.ranges.clear();
            for(int i = 0; i < scan.ranges.size(); i++) {
                r = scan.ranges[i];
                scan_w.ranges.push_back(r);
            }
        }

    }else {
        scan_w = scan;
    }
}

double ray_tracing(double x_, double y_, double anpha, nav_msgs::OccupancyGrid& map) {
    int id_x0, id_y0;
    int id_x1, id_y1;
    int id_x, id_y;
    bool steep;
    int x_step, y_step;
    int delta_x, delta_y;
    int err, delta_err;

    id_x0 = (x_ - map.info.origin.position.x)/map.info.resolution;
    id_y0 = (y_ - map.info.origin.position.y)/map.info.resolution;

    id_x1 = (x_ + range_max*cos(anpha) - map.info.origin.position.x)/map.info.resolution;
    id_y1 = (y_ + range_max*sin(anpha) - map.info.origin.position.y)/map.info.resolution;

    if(abs(id_y1 - id_y0) > abs(id_x1 - id_x0)) {
        steep = 1;
    }else {
        steep = 0;
    }
    if(steep) {
        swap(id_x0, id_y0);
        swap(id_x1, id_y1);
    }
    delta_x = abs(id_x1 - id_x0); delta_y = abs(id_y1 - id_y0);
    err =0; delta_err = delta_y;
    id_x = id_x0; id_y = id_y0;
    if(id_x0 < id_x1) {
        x_step = 1;
    }else {
        x_step = -1;
    }
    if(id_y0 < id_y1) {
        y_step = 1;
    }else {
        y_step = -1;
    }
    if(steep) {
        if(!map_valid(id_y, id_x, map) || map.data[id_x*map.info.width + id_y] != free)
            return map.info.resolution*sqrt(pow(id_y - id_y0, 2) + pow(id_x - id_x0, 2));
    }else {
        if(!map_valid(id_x, id_y, map) || map.data[id_y*map.info.width + id_x] != free)
            return map.info.resolution*sqrt(pow(id_x - id_x0, 2) + pow(id_y - id_y0, 2));
    }
    while(id_x != (id_x1 + x_step*1)) {
        id_x += x_step;
        err += delta_err;
        if(2*err >= delta_x) {
            id_y += y_step;
            err -= delta_x;
        }
        if(steep) {
            if(!map_valid(id_y, id_x, map) || map.data[id_x*map.info.width + id_y] != free)
                return map.info.resolution*sqrt(pow(id_y - id_y0, 2) + pow(id_x - id_x0, 2));
        }else {
            if(!map_valid(id_x, id_y, map) || map.data[id_y*map.info.width + id_x] != free)
                return map.info.resolution*sqrt(pow(id_x - id_x0, 2) + pow(id_y - id_y0, 2));
        }
    }
    return range_max; 
}

/** Likelihood field model*/
double likelihoodfield_model(pf_scan_t& scan_w, pf_vector_t& q, nav_msgs::OccupancyGrid& map, pf_vector_t& laser_pose, vector<double>& map_dist) {
    double a = z_hit/sqrt(2*M_PI*pow(sigma_hit, 2));
    double d_min;
    pf_vector_t pose_lidar, pose_endpoint;
    double beam_angle;
    pose_lidar.v[0] = q.v[0] + laser_pose.v[0]*cos(q.v[2]) - laser_pose.v[1]*sin(q.v[2]);
    pose_lidar.v[1] = q.v[1] + laser_pose.v[0]*sin(q.v[2]) + laser_pose.v[1]*cos(q.v[2]);
    double prob = 1;
    for(int i = 0; i < scan_w.ranges.size(); i++) {
        if(scan_w.ranges[i].v[0] < range_max && scan_w.ranges[i].v[0] >= range_min) {
            beam_angle = scan_w.ranges[i].v[1] + q.v[2] + laser_pose.v[2];
            pose_endpoint.v[0] = pose_lidar.v[0] + scan_w.ranges[i].v[0]*cos(beam_angle);
            pose_endpoint.v[1] = pose_lidar.v[1] + scan_w.ranges[i].v[0]*sin(beam_angle);
            
            int idx = (pose_endpoint.v[0] - map.info.origin.position.x)/map.info.resolution;
            int idy = (pose_endpoint.v[1] - map.info.origin.position.y)/map.info.resolution;
            int id_cell = idy*map.info.width + idx;
            if(map_valid(idx, idy, map)) {
                d_min = map_dist[id_cell];
            }else {
                d_min = max_occ_dist;
            }
            prob *= a*exp(-0.5*pow(d_min/sigma_hit, 2)) + + z_rand/range_max;
        }
    } 
    return prob;   
}

/** Beam sensor model*/
double beams_model(pf_scan_t& scan_w, pf_vector_t& q, nav_msgs::OccupancyGrid& map, pf_vector_t& laser_pose) {
    double prob = 1;
    double beam_angle;
    double range_;
    double p_hit, p_short, p_rand, p_max, p;
    double eta_hit, eta_short;
    double c, upper_bound, lower_bound;
    double x0, y0;
    x0 = q.v[0] + laser_pose.v[0]*cos(q.v[2]) - laser_pose.v[1]*sin(q.v[2]);
    y0 = q.v[1] + laser_pose.v[0]*sin(q.v[2]) + laser_pose.v[1]*cos(q.v[2]);
    for(int i = 0; i < scan_w.ranges.size(); i++) {
        beam_angle = scan_w.ranges[i].v[1] + q.v[2] + laser_pose.v[2];
        range_ = ray_tracing(x0, y0, beam_angle, map);

        c = 1.0/sqrt(2*sigma_hit*sigma_hit);
        upper_bound = range_max - range_;
        lower_bound = 0 - range_;

        eta_hit = 2/(erf(upper_bound*c) - erf(lower_bound*c));
        if(scan_w.ranges[i].v[0] >= 0 && scan_w.ranges[i].v[0] <= range_max) {
            p_hit = eta_hit*c/(sqrt(M_PI))*exp(-pow(c*(scan_w.ranges[i].v[0] - range_), 2));
            p_rand = 1/range_max;
        }else {
            p_hit = 0;
            p_rand = 0;
        }

        eta_short = 1.0/(1 - exp(-lamda_short*range_));
        if(scan_w.ranges[i].v[0] >= 0 && scan_w.ranges[i].v[0] <= range_) {
            p_short = eta_short*lamda_short*exp(-lamda_short*scan_w.ranges[i].v[0]);
        }else {
            p_short = 0;
        }
        if(scan_w.ranges[i].v[0] == range_max) {
            p_max = 1;
        }else {
            p_max = 0;
        }
        p = lamda_hit*p_hit + lamda_short*p_short + lamda_max*p_max + lamda_rand*p_rand;
        prob *= p;
    }
    return prob;
}
