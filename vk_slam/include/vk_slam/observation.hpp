#pragma once
#include "vk_slam/vk_slam.hpp"

bool scan_valid(double z) {
    return z < range_max && z >= range_min;
}

void dataCallback(const nav_msgs::Odometry& msg, const sensor_msgs::LaserScan& scan) {
    scan_t.ranges.clear();
    sl_vector_t ray_i;
    float inv_scan;
    if(inverted_laser) {
        inv_scan = -1.0;
    }else {
        inv_scan = 1.0;
    }
    for(int i = 0; i < scan.ranges.size(); i += throttle_scan) {
        if(scan_valid(scan.ranges[i])) {
            ray_i.v[0] = scan.ranges[i];
            ray_i.v[1] = inv_scan*(i*angle_increment - angle_max);
            scan_t.ranges.push_back(ray_i);
        } 
    }
    odom.pose.pose.position.x = msg.pose.pose.position.x;
    odom.pose.pose.position.y = msg.pose.pose.position.y;
    odom.pose.pose.orientation = msg.pose.pose.orientation;
    data_ = true;
}

/** Occupancy grid map - using Bresenham's Line Drawing Algorithm
    -1: un-known    0: free     100: occupied */
void ray_tracing(sl_node_t& node_i, nav_msgs::OccupancyGrid& map_t, vector<double>& log_map_t) {
    /** Lidar pose in map frame */
    double x, y, x_cell, y_cell;
    x = node_i.pose.v[0] + laser_pose_x*cos(node_i.pose.v[2]) - laser_pose_y*sin(node_i.pose.v[2]);
    y = node_i.pose.v[1] + laser_pose_x*sin(node_i.pose.v[2]) + laser_pose_y*cos(node_i.pose.v[2]);

    double d, anpha, l_inv;
    int idx_cell;

    int map_width, map_height;
    int num_beam = node_i.scan.ranges.size();
    double x_offset, y_offset, map_resolution;
    x_offset = map_t.info.origin.position.x;
    y_offset = map_t.info.origin.position.y;
    map_resolution = map_t.info.resolution;
    map_width = map_t.info.width;
    map_height = map_t.info.height;

    int del_x, del_y, e;
    int id_x1, id_x2, id_y1, id_y2, id_x, id_y;
    int x_step, y_step, p1, p2;
    id_x1 = (x - x_offset)/map_resolution + 1;
    id_y1 = (y - y_offset)/map_resolution + 1;
    for(int k = 0; k < num_beam; k ++) {
        anpha = node_i.scan.ranges[k].v[1] + node_i.pose.v[2] + laser_pose_theta;
        id_x2 = (x + node_i.scan.ranges[k].v[0]*cos(anpha) - x_offset)/map_resolution + 1;
        id_y2 = (y + node_i.scan.ranges[k].v[0]*sin(anpha) - y_offset)/map_resolution + 1;

        del_x = id_x2 - id_x1;
	    del_y = id_y2 - id_y1;
	    if (del_x < 0) del_x = -del_x;
	    if (del_y < 0) del_y = -del_y;
	    x_step = 1;
	    if (id_x2 < id_x1) x_step = -1;
	    y_step = 1;
	    if (id_y2 < id_y1) y_step = -1;
	    id_x = id_x1; id_y = id_y1;
        if (del_x > del_y) {
		    e = 2*del_y - del_x;
		    p1 = 2*(del_y - del_x);
		    p2 = 2*del_y;
		    for(int i = 0; i < del_x; i++) {
			    if(e >= 0) {
				    id_y += y_step;
				    e += p1;
			    } else {
				    e += p2;
                }
			    id_x += x_step;
			    idx_cell = (id_y - 1)*map_width + id_x - 1;
                x_cell = id_x*map_resolution - map_resolution/2 + x_offset;
                y_cell = id_y*map_resolution - map_resolution/2 + y_offset;
                d = sqrt(pow(x_cell - x, 2) + pow(y_cell - y, 2));
                if (d > min(range_max, node_i.scan.ranges[k].v[0] + map_resolution/2)) {
                    l_inv = l_0;
                } else if (node_i.scan.ranges[k].v[0] < range_max && fabs(d - node_i.scan.ranges[k].v[0]) < map_resolution/2) {
                    l_inv = l_occ;
                } else if (d <= node_i.scan.ranges[k].v[0]) {
                    l_inv = l_free;
                }
                if(idx_cell < map_t.data.size()) {
                    log_map_t[idx_cell] += l_inv - l_0;
                }else {
                    ROS_WARN("Overload map size!");
                }
		    }
	    } else {
		    e = 2*del_x - del_y;
		    p1 = 2*(del_x - del_y);
		    p2 = 2*del_x;
		    for(int i = 0; i < del_y; i++) {
			    if (e >= 0) {
				    id_x += x_step;
				    e += p1;
			    } else {
				    e += p2;
                }
			    id_y += y_step;
			    idx_cell = (id_y - 1)*map_width + id_x - 1;
                x_cell = id_x*map_resolution - map_resolution/2 + x_offset;
                y_cell = id_y*map_resolution - map_resolution/2 + y_offset;
                d = sqrt(pow(x_cell - x, 2) + pow(y_cell - y, 2));
                if (d > min(range_max, node_i.scan.ranges[k].v[0] + map_resolution/2)) {
                    l_inv = l_0;
                } else if (node_i.scan.ranges[k].v[0] < range_max && fabs(d - node_i.scan.ranges[k].v[0]) < map_resolution/2) {
                    l_inv = l_occ;
                } else if (d <= node_i.scan.ranges[k].v[0]) {
                    l_inv = l_free;
                }
                if(idx_cell < map_t.data.size()) {
                    log_map_t[idx_cell] += l_inv - l_0;
                }else {
                    ROS_WARN("Overload map size!");
                }
		    }
	    }
    }
}

void mapping(sl_graph_t& graph_t_, vector<double>& log_map_t, nav_msgs::OccupancyGrid& map_t, nav_msgs::Path& pose_graph_t) {
    int num_nodes = graph_t_.set_node_t.size();
    int num_cells = map_t.data.size();
    if(loop_closure_detected) {
        for (int i = 0; i < num_cells; i++) {
            log_map_t[i] = l_0;
        }
        for(int i = 0; i < num_nodes; i++) {
            ray_tracing(graph_t_.set_node_t[i], map_t, log_map_t);
        }
    }else {
        ray_tracing(graph_t_.set_node_t.back(), map_t, log_map_t);
    }
    for(int i = 0; i < num_cells; i++) {
        if (exp(log_map_t[i]) <= 0.333) {
            map_t.data[i] = free;
        } else if (exp(log_map_t[i]) >= 3.0) {
            map_t.data[i] = occupied;
        } else {
            map_t.data[i] = unknown;
        }
    }
    map_pub.publish(map_t);

    geometry_msgs::PoseStamped p;
    pose_graph_t.poses.clear();
    for(int i = 0; i < num_nodes; i++) {
        p.pose.position.x = graph_t_.set_node_t[i].pose.v[0];
        p.pose.position.y = graph_t_.set_node_t[i].pose.v[1];
        p.pose.position.z = 0.0;
        pose_graph_t.poses.push_back(p);
    }
    pose_graph_pub.publish(pose_graph_t);
}

/** Initial map and first node */
void init_slam(vector<double>& log_map_t, nav_msgs::OccupancyGrid& map_t, nav_msgs::Path& pose_graph_t) {
    map_t.header.frame_id = map_frame;
    map_t.info.resolution = map_resolution;
    map_t.info.width = map_width;
    map_t.info.height = map_height;
    map_t.info.origin.position.x = -0.5*map_width*map_resolution;
    map_t.info.origin.position.y = -0.5*map_height*map_resolution;
    map_t.info.origin.position.z = 0.0;
    map_t.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
    map_t.data.resize(map_t.info.width*map_t.info.height);

    pose_graph_t.header.frame_id = map_frame;
    int num_cells = map_t.data.size();
    log_map_t.clear();
    for (int i = 0; i < num_cells; i++) {
        log_map_t.push_back(l_0);
    }
}

bool update_node(sl_vector_t u_t[2]) {
    return sqrt(pow(u_t[1].v[0] - u_t[0].v[0], 2) + pow(u_t[1].v[1] - u_t[0].v[1], 2)) > min_trans || fabs(angle_diff(u_t[1].v[2], u_t[0].v[2])) > min_rot;
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
