#pragma once
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <random>
#include <iostream>
#include <math.h>
#include <vector>
#include <algorithm>

using namespace std;

struct vector_t {
    double v[2];
};

struct cell_map {
    int idx;
    int idy;
};

struct node_ {
    int id_node;
    int id_node_parent;
    cell_map cell;
    vector_t position;
    double past_cost;
    bool close;
};

struct node_t {
    node_ node;
    vector<node_> neighbor;
};

struct set_node_t {
    vector<node_t> vec_node_t;
};

#define occupied 100
#define free 0
#define unknown -1
#define inf 1e6

class Astar {
    private:
        int num_node;
        vector_t init_point_;
        vector_t goal_point_;
        set_node_t path_;
    public:
        Astar(vector_t init_point, vector_t goal_point);
        void astar_init(set_node_t& nodes, nav_msgs::OccupancyGrid& map);
        double cost(node_t& a, node_t& b);
        double heuristic_cost(node_t& a, nav_msgs::OccupancyGrid& map);
        void make_line(set_node_t& path);
        void neighbor_func(node_t& a, set_node_t& nodes, nav_msgs::OccupancyGrid& map);
        void least_square(set_node_t& path, nav_msgs::OccupancyGrid& map);
        bool collision_check(node_t& a, node_t& b, nav_msgs::OccupancyGrid& map);
        void smooth_func(set_node_t& path, nav_msgs::OccupancyGrid& map);
        void path_planning(node_t& init_node, node_t& goal_node, set_node_t& path, nav_msgs::OccupancyGrid& map);
        void astar_publisher(nav_msgs::Path& path);
        ~Astar();
};

nav_msgs::OccupancyGrid cost_map;
vector_t init_pose, goal_pose;

bool _map = false, _initpose = false, _goalpose = false;
double eta;

void mapCallback(const nav_msgs::OccupancyGrid& msg);
void initposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
void goalposeCallback(const geometry_msgs::PoseStamped& msg);
bool map_valid(nav_msgs::OccupancyGrid& map, int i, int j);