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
    - Define for cell value: -1: un-known
                            0: free
                            100: occupied */
void compute_logmap(int idx_x1, int idx_y1, int idx_x2, int idx_y2, nav_msgs::OccupancyGrid& map_t, double range_i, vector<double>& log_map_t) {
    int idx_cell = (idx_y2 - 1)*map_t.info.width + idx_x2 - 1;
    int l_inv;
    double d = map_t.info.resolution*sqrt(pow(idx_x1 - idx_x2, 2) + pow(idx_y1 - idx_y2, 2));
    if (d > min(range_max, range_i + map_t.info.resolution/2)) {
        l_inv = l_0;
    } else if (range_i < range_max && fabs(d - range_i) < map_t.info.resolution/2) {
        l_inv = l_occ;
    } else if (d <= range_i) {
        l_inv = l_free;
    }
    log_map_t[idx_cell] += l_inv - l_0;
}

void ray_tracing(sl_node_t& node_i, nav_msgs::OccupancyGrid& map_t, vector<double>& log_map_t) {
    /** Lidar pose in map frame */
    double x, y;
    x = node_i.pose.v[0] + laser_pose_x*cos(node_i.pose.v[2]) - laser_pose_y*sin(node_i.pose.v[2]);
    y = node_i.pose.v[1] + laser_pose_x*sin(node_i.pose.v[2]) + laser_pose_y*cos(node_i.pose.v[2]);

    int num_beam = node_i.scan.ranges.size();
    double x_offset, y_offset, map_resolution;
    x_offset = map_t.info.origin.position.x;
    y_offset = map_t.info.origin.position.y;
    map_resolution = map_t.info.resolution;

    int map_width, map_height;
    map_width = map_t.info.width;

    int idx_x1, idx_y1;
    int idx_x2, idx_y2;
    int idx_x, idx_y, P;
    int del_x, del_y;
    double d, anpha;

    idx_x1 = (x - x_offset)/map_resolution + 1;
    idx_y1 = (y - y_offset)/map_resolution + 1;
    for(int k = 0; k < num_beam; k ++) {
        anpha = node_i.scan.ranges[k].v[1] + node_i.pose.v[2] + laser_pose_theta;
        idx_x2 = (x + node_i.scan.ranges[k].v[0]*cos(anpha) - x_offset)/map_resolution + 1;
        idx_y2 = (y + node_i.scan.ranges[k].v[0]*sin(anpha) - y_offset)/map_resolution + 1;
        if (idx_x1 < idx_x2 && idx_y1 < idx_y2) {                               /** (1) */
            idx_x = idx_x1; idx_y = idx_y1;
            del_x = idx_x2 - idx_x1; del_y = idx_y2 - idx_y1;
            if(abs(del_x) == abs(del_y)) {
                while (idx_x <= idx_x2) {
                    compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                    idx_x++;
                    idx_y++;
                }
            }else if(abs(del_x) > abs(del_y)) {
                P = 2*del_y - del_x;
                while (idx_x <= idx_x2) {
                    compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                    if (P < 0) {
                        P += 2*del_y;
                    } else {
                        P += 2*del_y - 2*del_x;
                        idx_y++;
                    }
                    idx_x++;
                }
            }else {
                P = 2*del_x - del_y;
                while (idx_y <= idx_y2) {
                    compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                    if (P < 0) {
                        P += 2*del_x;
                    } else {
                        P += 2*del_x - 2*del_y;
                        idx_x++;
                    }
                    idx_y++;
                }
            }

        }else if (idx_x1 > idx_x2 && idx_y1 < idx_y2) {                     /** (2) */
            idx_x = idx_x1; idx_y = idx_y1;
            del_x = idx_x2 - idx_x1;
            del_y = idx_y2 - idx_y1;
            if(abs(del_x) == abs(del_y)) {
                while (idx_x >= idx_x2) {
                    compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                    idx_x--;
                    idx_y++;
                }
            } else if (abs(del_x) > abs(del_y)) {
                P = -2*del_y - del_x;
                while (idx_x >= idx_x2) {
                    compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                    if (P > 0) {
                        P = P - 2*del_y;
                    } else {
                        P = P - 2*del_y - 2*del_x;
                        idx_y++;
                    }
                    idx_x--;
                }
            } else if (abs(del_x) < abs(del_y)) {
                P = -2*del_x - del_y;
                while (idx_y <= idx_y2) {
                    compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                    if (P < 0) {
                        P = P - 2*del_x;
                    } else {
                        P = P - 2*del_x - 2*del_y;
                        idx_x--;
                    }
                    idx_y++;
                }
            }  
        }else if (idx_x1 == idx_x2 && idx_y1 < idx_y2) {                    /** (3) */
            idx_x = idx_x1;
            idx_y = idx_y1;
            while (idx_y <= idx_y2) {
                compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                idx_y++;
            }
        }else if (idx_x1 < idx_x2 && idx_y1 > idx_y2) {                     /** (4) */
            idx_x = idx_x2;
            idx_y = idx_y2;
            del_x = idx_x1 - idx_x2;
            del_y = idx_y1 - idx_y2;
            if (abs(del_x) == abs(del_y)) {
                while (idx_x >= idx_x1) {
                    compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                    idx_x--;
                    idx_y++;
                }
            } else if (abs(del_x) > abs(del_y)) {
                P = -2*del_y - del_x;
                while (idx_x >= idx_x1) {
                    compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                    if (P > 0) {
                        P = P - 2*del_y;
                    } else {
                        P = P - 2*del_y - 2*del_x;
                        idx_y++;
                    }
                    idx_x--;
                }
            } else if (abs(del_x) < abs(del_y)) {
                P = -2*del_x - del_y;
                while (idx_y <= idx_y1) {
                    compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                    if (P < 0) {
                        P = P - 2*del_x;
                    } else {
                        P = P - 2*del_x - 2*del_y;
                        idx_x--;
                    }
                    idx_y++;
                }
            }
        }else if (idx_x1 == idx_x2 && idx_y1 > idx_y2) {                    /** (5) */
            idx_x = idx_x1;
            idx_y = idx_y1;
            while (idx_y >= idx_y2) {
                compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                idx_y--;
            }
        }else if (idx_x1 > idx_x2 && idx_y1 > idx_y2) {                     /** (6) */
            idx_x = idx_x2;
            idx_y = idx_y2;
            del_x = idx_x1 - idx_x2;
            del_y = idx_y1 - idx_y2;
            if (abs(del_x) == abs(del_y)) {
                while (idx_x <= idx_x1) {
                    compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                    idx_x++;
                    idx_y++;
                }
            } else if (abs(del_x) > abs(del_y)) {
                P = 2*del_y - del_x;
                while (idx_x <= idx_x1) {
                    compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                    if (P < 0) {
                        P += 2*del_y;
                    } else {
                        P += 2*del_y - 2*del_x;
                        idx_y++;
                    }
                    idx_x++;
                }
            } else {
                P = 2*del_x - del_y;
                while (idx_y <= idx_y1) {
                    compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                    if (P < 0) {
                        P += 2*del_x;
                    } else {
                        P += 2*del_x - 2*del_y;
                        idx_x++;
                    }
                    idx_y++;
                }
            }
        }else if (idx_x1 < idx_x2 && idx_y1 == idx_y2) {                    /** (7) */
            idx_x = idx_x1;
            idx_y = idx_y1;
            while(idx_x <= idx_x2) {
                compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                idx_x++;
            }
        }else if (idx_x1 == idx_x2 && idx_y1 == idx_y2) {                   /** (8) */
            idx_x = idx_x1;
            idx_y = idx_y1;
            compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
        }else { /** idx_x1 > idx_x2 && idx_y1 == idx_y2 */                  /** (9) */
            idx_x = idx_x1;
            idx_y = idx_y1;
            while (idx_x >= idx_x2) {
                compute_logmap(idx_x1, idx_y1, idx_x, idx_y, map_t, node_i.scan.ranges[k].v[0], log_map_t);
                idx_x--;
            }
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
