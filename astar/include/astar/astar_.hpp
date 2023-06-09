#pragma once
#include "astar/astar.hpp"

void Astar::mapCallback(const nav_msgs::OccupancyGrid& msg){
    cost_map.header.frame_id = msg.header.frame_id;
    cost_map.info.width = msg.info.width;
    cost_map.info.height = msg.info.height;
    cost_map.data = msg.data;
    cost_map.info.resolution = msg.info.resolution;
    cost_map.info.origin.position.x = msg.info.origin.position.x;
    cost_map.info.origin.position.y = msg.info.origin.position.y;
    _map = true;
}

void Astar::initposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg) {
    init_point_.v[0] = msg.pose.pose.position.x;
    init_point_.v[1] = msg.pose.pose.position.y;
    _initpose = true;
}

void Astar::goalposeCallback(const geometry_msgs::PoseStamped& msg) {
    goal_point_.v[0] = msg.pose.position.x;
    goal_point_.v[1] = msg.pose.position.y;
    _goalpose = true;
}

bool Astar::map_valid(nav_msgs::OccupancyGrid& map, int i, int j) {
    return i >= 0 && i < map.info.width && j >=0 && j < map.info.height;
}

int Astar::cell_id(vector_t init_point, nav_msgs::OccupancyGrid& map) {
    int idx = (init_point.v[0] - map.info.origin.position.x)/map.info.resolution;
    int idy = (init_point.v[1] - map.info.origin.position.y)/map.info.resolution;
    if(map_valid(map, idx, idy)) {
        return idy*map.info.width + idx;
    }else {
        ROS_ERROR("Invalid initial pose!");
        return -1;
    }
}

void Astar::astar_init(set_node_t& nodes, nav_msgs::OccupancyGrid& map) {
    nodes.vec_node_t.clear();
    node_t n;
    int ID = cell_id(init_point_, map);
    for(int i = 0; i < map.data.size(); i++) {
        n.node.id_node = i;
        n.node.cell.idy = i/map.info.width;
        n.node.cell.idx = i - n.node.cell.idy * map.info.width;
        n.node.position.v[0] = n.node.cell.idx * map.info.resolution + map.info.origin.position.x;
        n.node.position.v[1] = n.node.cell.idy * map.info.resolution + map.info.origin.position.y;
        n.node.close = false;
        if(i != ID) {
            n.node.past_cost = inf;
        }else {
            n.node.past_cost = 0.0;
        }
        nodes.vec_node_t.push_back(n);
    }
}

double Astar::cost(node_& a, node_& b) {
    if(a.cell.idx == b.cell.idx || a.cell.idy == b.cell.idy) {
        return 1.0;
    }else {
        return sqrt(2.0);
    }
}

double Astar::heuristic_cost(node_& a, nav_msgs::OccupancyGrid& map) {
    cell_map goal_cell;
    goal_cell.idx = (goal_point_.v[0] - map.info.origin.position.x)/map.info.resolution;
    goal_cell.idy = (goal_point_.v[1] - map.info.origin.position.x)/map.info.resolution;
    return eta*sqrt(pow(a.cell.idx- goal_cell.idx, 2) + pow(a.cell.idy - goal_cell.idy, 2));
}

bool Astar::collision_check(node_t& a, node_t& b, nav_msgs::OccupancyGrid& map) {
    double eps;
    int idx, idy;
    double x_step, y_step;
    vector<node_t> vec_A;
    vec_A.clear();
    vec_A.push_back(a);
    vec_A.push_back(b);
    do {
        x_step = 0.5*(vec_A[1].node.position.v[0] - vec_A[0].node.position.v[0]);
        y_step = 0.5*(vec_A[1].node.position.v[1] - vec_A[0].node.position.v[1]);
        vec_A.resize(2*vec_A.size() - 1);
        for(int i = 0; i < vec_A.size(); i++) {
            vec_A[i].node.position.v[0] = vec_A[0].node.position.v[0] + i*x_step;
            vec_A[i].node.position.v[1] = vec_A[0].node.position.v[1] + i*y_step;
        }
        eps = pow(vec_A[1].node.position.v[0] - vec_A[0].node.position.v[0], 2) + pow(vec_A[1].node.position.v[1] - vec_A[0].node.position.v[1], 2);
        for(int i = 0; i < vec_A.size(); i++) {
            idx = (vec_A[i].node.position.v[0] - map.info.origin.position.x)/map.info.resolution;
            idy = (vec_A[i].node.position.v[1] - map.info.origin.position.y)/map.info.resolution;
            if(!map_valid(map, idx, idy) || map.data[idy*map.info.width + idx] != free) {
                return true;
            }
        }
    }while(eps > 1e-4);
    return false;
}

void Astar::neighbor_func(node_t& a, set_node_t& nodes, nav_msgs::OccupancyGrid& map) {
    a.neighbor.clear();
    for(int i = a.node.cell.idx - 1; i <= a.node.cell.idx + 1; i++) {
        if(i >= 0 && i < map.info.width) {
            for(int j = a.node.cell.idy - 1; j <= a.node.cell.idy + 1; j++) {
                if(j >= 0 && j < map.info.height) {
                    int ID = j*map.info.width + i;
                    if(map.data[ID] == free && nodes.vec_node_t[ID].node.close == false && collision_check(a, nodes.vec_node_t[ID], map) == false) {
                        a.neighbor.push_back(nodes.vec_node_t[ID].node);
                    }
                }
            }
        }
    }
}

void Astar::make_line(set_node_t& path) {
    set_node_t tmp;
    tmp = path;
    reverse(tmp.vec_node_t.begin(), tmp.vec_node_t.end());
    path.vec_node_t.clear();
    path.vec_node_t.push_back(tmp.vec_node_t[0]);
    double a[2], b[2];
    if(tmp.vec_node_t.size() > 2) {
        for(int i = 0; i < tmp.vec_node_t.size()-2; i++) {
            a[0] = tmp.vec_node_t[i+1].node.position.v[0] - tmp.vec_node_t[i].node.position.v[0];
            a[1] = tmp.vec_node_t[i+1].node.position.v[1] - tmp.vec_node_t[i].node.position.v[1];

            b[0] = tmp.vec_node_t[i+2].node.position.v[0] - tmp.vec_node_t[i].node.position.v[0];
            b[1] = tmp.vec_node_t[i+2].node.position.v[1] - tmp.vec_node_t[i].node.position.v[1];

            double k = fabs((a[0]*b[0] + a[1]*b[1])/(sqrt(pow(a[0], 2) + pow(a[1], 2))*sqrt(pow(b[0], 2) + pow(b[1], 2))));
            if(k != 1) {
                path.vec_node_t.push_back(tmp.vec_node_t[i+1]);
            }else if(i == tmp.vec_node_t.size() - 3) {
                path.vec_node_t.push_back(tmp.vec_node_t[i]);
                path.vec_node_t.push_back(tmp.vec_node_t[i+1]);
                path.vec_node_t.push_back(tmp.vec_node_t[i+2]);
            }
        }
    }else {
        path.vec_node_t.push_back(tmp.vec_node_t[1]);
    }
}

void Astar::least_square(set_node_t& path, nav_msgs::OccupancyGrid& map) {
    set_node_t tmp;
    tmp = path;
    reverse(tmp.vec_node_t.begin(), tmp.vec_node_t.end());
    path.vec_node_t.clear();
    path.vec_node_t.push_back(tmp.vec_node_t[0]);
    int k = 0;
    for(int i = 0; i < tmp.vec_node_t.size(); i++) {
        double sum_x = 0, sum_y = 0, sum_x2 = 0, sum_xy = 0;
        for(int j = k ; j < i; j++) {
            sum_x += tmp.vec_node_t[j].node.position.v[0];
            sum_y += tmp.vec_node_t[j].node.position.v[1];
            sum_x2 += tmp.vec_node_t[j].node.position.v[0] * tmp.vec_node_t[j].node.position.v[0];
            sum_xy += tmp.vec_node_t[j].node.position.v[0] * tmp.vec_node_t[j].node.position.v[1];
        }
        int n = i - k;
        double a = (sum_xy/n - sum_x*sum_y/pow(n, 2))/(sum_x2/(n) - pow(sum_x/n, 2));
        double b = (sum_y - a*sum_x)/n;
        node_t n_k = tmp.vec_node_t[i];
        double d = fabs(a*n_k.node.position.v[0] - n_k.node.position.v[1] + b)/sqrt(a*a + 1);
        if(d > sqrt(2)*map.info.resolution){
            path.vec_node_t.push_back(tmp.vec_node_t[i-1]);
            k = i-2;
        }
    }
    path.vec_node_t.push_back(tmp.vec_node_t.back());
}

void Astar::smooth_func(set_node_t& path, nav_msgs::OccupancyGrid& map) {
    set_node_t tmp;
    tmp = path;
    path.vec_node_t.clear();
    node_t node_goal, node_init;
    node_init = tmp.vec_node_t.front();
    node_goal = tmp.vec_node_t.back();
    path.vec_node_t.push_back(node_goal);
    int K = tmp.vec_node_t.size();
    do {
        for(int i = 0; i < K; i++) {
            if(!collision_check(node_goal, tmp.vec_node_t[i], map)) {
                node_goal = tmp.vec_node_t[i];
                path.vec_node_t.push_back(tmp.vec_node_t[i]);
                K = i;
                break; 
            }
        }
    }while(node_goal.node.id_node != node_init.node.id_node);
    reverse(path.vec_node_t.begin(), path.vec_node_t.end());
}

void Astar::path_planning(set_node_t& path, nav_msgs::OccupancyGrid& map) {
    set_node_t all_nodes;
    astar_init(all_nodes, map);
    vector<node_> OPEN;
    set_node_t CLOSED;
    int ID_init_node = cell_id(init_point_, map);
    int ID_goal_node = cell_id(goal_point_, map);
    OPEN.push_back(all_nodes.vec_node_t[ID_init_node].node);
    node_t current_node;
    while(OPEN.size() != 0) {
        current_node.node = OPEN[0];
        OPEN.erase(OPEN.begin());
        CLOSED.vec_node_t.push_back(current_node);
        all_nodes.vec_node_t[current_node.node.id_node].node.close = true;
        if(current_node.node.id_node == ID_goal_node) {
            ROS_INFO("Path planning is successful!");
            _success = true;
            break;
        }
        neighbor_func(current_node, all_nodes, map);
        for(int i = 0; i < current_node.neighbor.size(); i++) {
            double tentative_past_cost = current_node.node.past_cost + cost(current_node.node, current_node.neighbor[i]);
            if(tentative_past_cost < current_node.neighbor[i].past_cost) {
                current_node.neighbor[i].past_cost = tentative_past_cost;
                all_nodes.vec_node_t[current_node.neighbor[i].id_node].node.past_cost = tentative_past_cost;
                current_node.neighbor[i].id_node_parent = current_node.node.id_node;
                all_nodes.vec_node_t[current_node.neighbor[i].id_node].node.id_node_parent = current_node.node.id_node;
                OPEN.push_back(current_node.neighbor[i]);
            }
        }
        for(int i = 0; i < OPEN.size() - 1; i++) {
            for(int j = i+1; j < OPEN.size(); j++) {
                double total_cost1 = OPEN[i].past_cost + heuristic_cost(OPEN[i], map);
                double total_cost2 = OPEN[j].past_cost + heuristic_cost(OPEN[j], map);
                if(total_cost1 > total_cost2) {
                    node_ temp = OPEN[i];
                    OPEN[i] = OPEN[j];
                    OPEN[j] = temp;
                }
            }
        }
    }
    if(_success) {
        node_t node_last;
        node_t node_goal = CLOSED.vec_node_t.back();
        path.vec_node_t.clear();
        path.vec_node_t.push_back(node_goal);
        node_last = node_goal;
        while(node_last.node.id_node != ID_init_node) {
            node_last = all_nodes.vec_node_t[node_last.node.id_node_parent];
            path.vec_node_t.push_back(node_last);
        }
        make_line(path);
        // // least_square(path, map);
        smooth_func(path, map);
    }
}

void Astar::astar_publisher() {
    nav_msgs::Path path_Astar;
    geometry_msgs::PoseStamped p;
    path_Astar.header.frame_id = map_frame;
    path_Astar.header.stamp = ros::Time::now();
    path_Astar.poses.clear();
    for(int i = 0; i < path_.vec_node_t.size(); i++) {
        p.pose.position.x = path_.vec_node_t[i].node.position.v[0];
        p.pose.position.y = path_.vec_node_t[i].node.position.v[1];
        p.pose.position.z = 0.0;
        path_Astar.poses.push_back(p);
    }
    path_pub.publish(path_Astar);
}