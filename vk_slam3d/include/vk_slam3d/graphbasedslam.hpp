#pragma once
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include "vk_slam3d/odometrymodel.hpp"
#include "vk_slam3d/scanmatcher.hpp"
#include "vk_slam3d/adaptivethreshold.hpp"
#include "vk_slam3d/preprocessing.hpp"


namespace slam {

struct Node {
    Eigen::Vector6d pose;
    std::vector<Eigen::Vector3d> scan;
    Eigen::Matrix6d inverse_covariance;
    int idx;
};

struct Edge {
    Eigen::Vector6d z;
    Eigen::Matrix6d inverse_covariance;
    int i;
    int j;
};

struct Graph {
    std::vector<Edge> edges;
    std::vector<Node> nodes;
};

class GraphBasedSlam {
    private:
        ros::Publisher map_pub;
        ros::Publisher pose_graph_pub;
        ros::Publisher point_cloud_pub;

        ros::Subscriber laser_scan_sub;
        ros::Subscriber odom_sub;
        typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> MySyncPolicy;
        void dataCallback(const nav_msgs::Odometry& msg1, const sensor_msgs::PointCloud2& msg2);

    public:
        std::string odom_frame;
        std::string map_frame;
        std::string base_frame;
        std::string model_type;

        double frequency;
        double anpha;  /* anpha = [0; 1.0] */
        double beta;   /* beta = [1.0; 2.0] */
        double voxel_size;
        int max_points_per_voxel;
        double min_trans;
        double min_rot;
        double min_range;
        double max_range;
        bool received_data;

        double initial_threshold;
        double min_motion;

        Eigen::Vector6d odom;
        std::vector<Eigen::Vector3d> scan;
        std::vector<Eigen::Vector3d> icpscan;

        visualization_msgs::Marker marker_node;
        visualization_msgs::Marker marker_edge;
        visualization_msgs::MarkerArray marker_graph;

        GraphBasedSlam(ros::NodeHandle nh);
        void init();
        void visualization(Graph& graph, VoxelHashMap::Ptr voxel_map);

};

}