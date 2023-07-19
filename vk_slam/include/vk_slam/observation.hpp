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
            ray_i.v[1] = inv_scan*(i*throttle_scan * angle_increment - angle_max);
            scan_t.ranges.push_back(ray_i);
        } 
    }
    odom.pose.pose.position.x = msg.pose.pose.position.x;
    odom.pose.pose.position.y = msg.pose.pose.position.y;
    odom.pose.pose.orientation = msg.pose.pose.orientation;
    data_ = true;
}

/** Occupancy grid map - using Bresenham's Line Drawing Algorithm 
    - Define for cell value: -1: un-known
                            0: free
                            100: occupied */
void ray_tracing(sl_node_t& node_i, nav_msgs::OccupancyGrid& map_t, vector<double>& log_map_t) {
    /** Lidar pose in map frame */
    double x, y;
    x = node_i.pose.v[0] + laser_pose_x*cos(node_i.pose.v[2]) - laser_pose_y*sin(node_i.pose.v[2]);
    y = node_i.pose.v[1] + laser_pose_x*sin(node_i.pose.v[2]) + laser_pose_y*cos(node_i.pose.v[2]);

    cout << node_i.pose.v[0] << " " << node_i.pose.v[1] << " " << node_i.pose.v[2] << endl;
    int num_beam = node_i.scan.ranges.size();
    double x_offset, y_offset, map_resolution;
    x_offset = map_t.info.origin.position.x;
    y_offset = map_t.info.origin.position.y;
    map_resolution = map_t.info.resolution;

    int map_width, map_height;
    map_width = map_t.info.width;
    map_height = map_t.info.height;

    int id_x0, id_y0;
    int id_x1, id_y1;
    int id_x, id_y;
    bool steep;
    int x_step, y_step;
    int delta_x, delta_y;
    int err, delta_err;

    id_x0 = (x - x_offset)/map_resolution;
    id_y0 = (y - y_offset)/map_resolution;
    double d, anpha, l_inv;
    int id_cell;

    for(int k = 0; k < num_beam; k ++) {
        anpha = node_i.scan.ranges[k].v[1] + node_i.pose.v[2] + laser_pose_theta;
        id_x1 = (x + node_i.scan.ranges[k].v[0]*cos(anpha) - x_offset)/map_resolution;
        id_y1 = (y + node_i.scan.ranges[k].v[0]*sin(anpha) - y_offset)/map_resolution;
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
        err = 0; delta_err = delta_y;
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
        while(id_x != (id_x1 + x_step*1)) {
            id_x += x_step;
            err += delta_err;
            if(2*err >= delta_x) {
                id_y += y_step;
                err -= delta_x;
            }
            if(steep) {
                id_cell = id_y*map_width + id_x;
                d = map_resolution*sqrt(pow(id_y - id_y0, 2) + pow(id_x - id_x0, 2));
            }else {
                id_cell = id_x*map_width + id_y;
                d = map_resolution*sqrt(pow(id_x - id_x0, 2) + pow(id_y - id_y0, 2));
            }
            if (d > min(range_max, node_i.scan.ranges[k].v[0] + map_resolution/2)) {
                l_inv = l_0;
            } else if (node_i.scan.ranges[k].v[0] < range_max && fabs(d - node_i.scan.ranges[k].v[0]) < map_resolution/2) {
                l_inv = l_occ;
            } else if (d <= node_i.scan.ranges[k].v[0]) {
                l_inv = l_free;
            }
            log_map_t[id_cell] += l_inv - l_0;
        }
    }

}

void mapping(sl_graph_t& graph_t_, vector<double>& log_map_t, nav_msgs::OccupancyGrid& map_t, nav_msgs::Path& pose_graph_t) {
    int num_nodes = graph_t_.set_node_t.size();
    // for(int i = 0; i < num_nodes; i++) {
        ray_tracing(graph_t_.set_node_t.back(), map_t, log_map_t);
    // }
    int num_cells = map_t.data.size();
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
