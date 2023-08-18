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
    for(int i = 0; i < 540; i += throttle_scan) {
        if(scan_valid(scan.ranges[i+135])) {
            ray_i.v[0] = scan.ranges[i+135];
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
    double x, y, theta, x_cell, y_cell;
    x = node_i.pose.v[0] + laser_pose_x*cos(node_i.pose.v[2]) - laser_pose_y*sin(node_i.pose.v[2]);
    y = node_i.pose.v[1] + laser_pose_x*sin(node_i.pose.v[2]) + laser_pose_y*cos(node_i.pose.v[2]);
    theta = normalize(node_i.pose.v[2] + laser_pose_theta);

    double d, anpha, l_inv;
    int idx_cell;
    double phi;

    int map_width, map_height;
    int num_beam = node_i.scan.ranges.size();
    double x_offset, y_offset, map_resolution;
    x_offset = map_t.info.origin.position.x;
    y_offset = map_t.info.origin.position.y;
    map_resolution = map_t.info.resolution;
    map_width = map_t.info.width;
    map_height = map_t.info.height;

    const int kernel = 20;
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
                phi = atan2(y_cell - y, x_cell - x) - theta;
                int k_min = k;
                for(int j = k - kernel; j < k + kernel; j++) {
                    if(j >= 0 && j < num_beam) {
                        if(fabs(phi - node_i.scan.ranges[j].v[1]) < fabs(phi - node_i.scan.ranges[k_min].v[1])) {
                            k_min = j;
                        }
                    }
                }
                if (d > min(range_max, node_i.scan.ranges[k_min].v[0] + map_resolution/2) || fabs(phi - node_i.scan.ranges[k_min].v[1]) > 0.5*angle_increment) {
                    l_inv = l_0;
                } else if (node_i.scan.ranges[k_min].v[0] < range_max && fabs(d - node_i.scan.ranges[k_min].v[0]) < map_resolution/2) {
                    l_inv = l_occ;
                } else if (d <= node_i.scan.ranges[k_min].v[0]) {
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
                phi = atan2(y_cell - y, x_cell - x) - theta;
                int k_min = k;
                for(int j = k - kernel; j < k + kernel; j++) {
                    if(j >= 0 && j < num_beam) {
                        if(fabs(phi - node_i.scan.ranges[j].v[1]) < fabs(phi - node_i.scan.ranges[k_min].v[1])) {
                            k_min = j;
                        }
                    }
                }
                if (d > min(range_max, node_i.scan.ranges[k_min].v[0] + map_resolution/2) || fabs(phi - node_i.scan.ranges[k_min].v[1]) > 0.5*angle_increment) {
                    l_inv = l_0;
                } else if (node_i.scan.ranges[k_min].v[0] < range_max && fabs(d - node_i.scan.ranges[k_min].v[0]) < map_resolution/2) {
                    l_inv = l_occ;
                } else if (d <= node_i.scan.ranges[k_min].v[0]) {
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

void mapping(sl_graph_t& graph_t_, vector<double>& log_map_t, nav_msgs::OccupancyGrid& map_t) {
    mu.lock();
    int num_nodes = graph_t_.set_node_t.size();
    int num_cells = map_t.data.size();
    if(optimized) {
        ROS_INFO("Reloading map ...!");
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
    mu.unlock();
}

void pose_graph_visualization(sl_graph_t& graph_t_, visualization_msgs::Marker& node, visualization_msgs::Marker& edge, visualization_msgs::MarkerArray& SetOfMarker) {
    int num_nodes = graph_t_.set_node_t.size();
    int num_edges = graph_t_.set_edge_t.size();
    int id = 0;
    SetOfMarker.markers.clear();
    for(int i = 0; i < num_nodes; i++) {
        node.id = id;
        node.pose.position.x = graph_t_.set_node_t[i].pose.v[0];
        node.pose.position.y = graph_t_.set_node_t[i].pose.v[1];
        SetOfMarker.markers.push_back(node);
        id++;
    }
    
    int id_i, id_j; // Edge ij
    for(int j = 0; j < num_edges; j++) {
        edge.id = id;
        edge.points.clear();
        geometry_msgs::Point p;

        id_i = graph_t_.set_edge_t[j].i;
        id_j = graph_t_.set_edge_t[j].j;

        p.x = graph_t_.set_node_t[id_i].pose.v[0];
        p.y = graph_t_.set_node_t[id_i].pose.v[1];
        edge.points.push_back(p);

        p.x = graph_t_.set_node_t[id_j].pose.v[0];
        p.y = graph_t_.set_node_t[id_j].pose.v[1];
        edge.points.push_back(p);
        SetOfMarker.markers.push_back(edge);
        id++;
    }
    pose_graph_pub.publish(SetOfMarker);
}

/** Initial map and first node */
void init_slam(vector<double>& log_map_t, nav_msgs::OccupancyGrid& map_t, visualization_msgs::Marker& node, visualization_msgs::Marker& edge, visualization_msgs::MarkerArray& SetOfMarker, int color) {
    map_t.header.frame_id = map_frame;
    map_t.info.resolution = map_resolution;
    map_t.info.width = map_width;
    map_t.info.height = map_height;
    map_t.info.origin.position.x = -0.5*map_width*map_resolution;
    map_t.info.origin.position.y = -0.5*map_height*map_resolution;
    map_t.info.origin.position.z = 0.0;
    map_t.info.origin.orientation = tf::createQuaternionMsgFromYaw(0.0);
    map_t.data.resize(map_t.info.width*map_t.info.height);

    int num_cells = map_t.data.size();
    log_map_t.clear();
    for (int i = 0; i < num_cells; i++) {
        log_map_t.push_back(l_0);
    }

    /* Node color - red*/
    node.header.frame_id = "map";
    node.header.stamp = ros::Time::now();
    node.action = visualization_msgs::Marker::ADD;
    node.type = visualization_msgs::Marker::SPHERE;
    node.id = 0;
    node.ns = "vk_slam_node";
    node.scale.x = 0.1;
    node.scale.y = 0.1;
    node.scale.z = 0.1;
    node.pose.orientation.x = 0;
    node.pose.orientation.y = 0;
    node.pose.orientation.z = 0.0;
    node.pose.orientation.w = 1.0;
    if(color == 0)
    {
        node.color.r = 1.0;
        node.color.g = 0.0;
        node.color.b = 0.0;
    } else {
        node.color.r = 0.0;
        node.color.g = 1.0;
        node.color.b = 0.0;
    }

    node.color.a = 1.0;
    node.lifetime = ros::Duration(0);

    /* Edges - blue */
    edge.header.frame_id = "map";
    edge.header.stamp = ros::Time::now();
    edge.action = visualization_msgs::Marker::ADD;
    edge.type = visualization_msgs::Marker::LINE_STRIP;
    edge.id = 0;
    edge.scale.x = 0.02;
    edge.scale.y = 0.02;
    edge.scale.z = 0.02;
    edge.ns = "vk_slam_edge";

    edge.pose.orientation.x = 0;
    edge.pose.orientation.y = 0;
    edge.pose.orientation.z = 0.0;
    edge.pose.orientation.w = 1.0;
    if(color == 0)
    {
        edge.color.r = 0.0;
        edge.color.g = 0.0;
        edge.color.b = 1.0;
    } else {
        edge.color.r = 1.0;
        edge.color.g = 0.0;
        edge.color.b = 1.0;
    }
    edge.color.a = 1.0;
    edge.lifetime = ros::Duration(0);
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

void printf_matrix(sl_matrix_t& A) {
    cout << "[";
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            cout << A.m[i][j] << "  " ;
        }
        cout << endl;
    }
    cout << "]" << endl;
}

/* Trimming ICP */
// double gold_rate(sl_point_cloud_t& pcl_ref, sl_point_cloud_t& pcl_cur_w, vector<sl_corr_t>& cores) {
//     double s_min = 0.4, s_max = 1.0;
//     double s1, s2, f1, f2;
//     double phi = (sqrt(5) - 1)/2;
//     int lamda = 2;
//     s1 = s_max - phi*(s_max - s_min); s2 = s_min + phi*(s_max - s_min);
//     while(fabs(s_max - s_min) > 1e-3) {
//         f1 = compute_sum_error_ICP(pcl_ref, pcl_cur_w, cores, s1)/(pow(s1, 1+lamda));
//         f2 = compute_sum_error_ICP(pcl_ref, pcl_cur_w, cores, s2)/(pow(s2, 1+lamda));
//         if(f1 < f2) {
//             s_max = s2;
//         }else {
//             s_min = s1;
//         }
//         s1 = s_max - phi*(s_max - s_min);
//         s2 = s_min + phi*(s_max - s_min);
//     }
//     return (s_min + s_max)/2;
// }
