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
        ros::Publisher path_pub;
        ros::Subscriber map_sub;
        ros::Subscriber initpose_sub;
        ros::Subscriber goalpose_sub;

        nav_msgs::OccupancyGrid cost_map;
        vector_t init_point_;
        vector_t goal_point_;
        set_node_t path_;

        bool _map;
        bool _initpose = true;
        bool _goalpose;
        double eta;
        bool _success = false;
        string map_frame;

        void mapCallback(const nav_msgs::OccupancyGrid& msg);
        void initposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
        void goalposeCallback(const geometry_msgs::PoseStamped& msg);
        bool map_valid(nav_msgs::OccupancyGrid& map, int i, int j);

        int cell_id(vector_t init_point, nav_msgs::OccupancyGrid& map);
        void astar_init(set_node_t& nodes, nav_msgs::OccupancyGrid& map);
        double cost(node_& a, node_& b);
        double heuristic_cost(node_& a, nav_msgs::OccupancyGrid& map);
        void make_line(set_node_t& path);
        void neighbor_func(node_t& a, set_node_t& nodes, nav_msgs::OccupancyGrid& map);
        void least_square(set_node_t& path, nav_msgs::OccupancyGrid& map);
        bool collision_check(node_t& a, node_t& b, nav_msgs::OccupancyGrid& map);
        void smooth_func(set_node_t& path, nav_msgs::OccupancyGrid& map);
        void path_planning(set_node_t& path, nav_msgs::OccupancyGrid& map);
        void astar_publisher();

        double publish_frequency;
    public:
        Astar(ros::NodeHandle* nh) {
            path_pub = nh->advertise<nav_msgs::Path>("path", 10);
            map_sub = nh->subscribe("map", 10, &Astar::mapCallback, this);
            initpose_sub = nh->subscribe("init_pose", 10, &Astar::initposeCallback, this);
            goalpose_sub = nh->subscribe("move_base_simple/goal", 10, &Astar::goalposeCallback, this);

            if(!ros::param::get("~eta", eta))
                eta = 2.0;
            if(!ros::param::get("~publish_frequency", publish_frequency))
                publish_frequency = 20;
            if(!ros::param::get("~map_frame", map_frame))
                map_frame = "map";
            
            ros::Rate rate(publish_frequency);
            while(ros::ok()) {
                ros::spinOnce();
                if(_map && _initpose && _goalpose) {
                    path_planning(path_, cost_map);
                    if(_success) {
                        astar_publisher();
                        _goalpose = false;
                    }else {
                        ROS_ERROR("Path planning is fail!");
                    }
                }
                rate.sleep();
            }
        };
        ~Astar() {};
};




