#include "vk_slam3d/graphbasedslam.hpp"
namespace slam {

GraphBasedSlam::GraphBasedSlam(ros::NodeHandle nh) {
    init();
    map_pub = nh.advertise<sensor_msgs::PointCloud>("map", 1);
    pose_graph_pub = nh.advertise<visualization_msgs::MarkerArray>("graph", 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> laser_scan_sub(nh, "velodyne_points", 1);
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, laser_scan_sub);
    sync.registerCallback(&GraphBasedSlam::dataCallback, this);
    
    VoxelHashMap::Ptr voxel_map (new VoxelHashMap(voxel_size, max_points_per_voxel));
    AdaptiveThreshold adaptive_threshold(initial_threshold, min_motion, max_range);
    ScanMatcher scanmatcher;
    Graph graph;

    Eigen::Vector6d odom_t_1;
    Eigen::Vector6d odom_t;
    Eigen::Vector6d u_t_1;
    Eigen::Vector6d u_t;
    Eigen::Vector6d robot_pose;
    Node node_t;
    Edge edge_t;
    bool first_time = true;
    double sigma;

    ros::Rate rate(frequency);
    while(ros::ok()) {
        ros::spinOnce();
        if(received_data) {
            odom_t = odom;
            if(first_time) {
                robot_pose = odom_t;
                node_t.pose = robot_pose;
                node_t.scan = scan;
                node_t.idx = 0;
                graph.nodes.push_back(node_t);
                Eigen::Matrix4d T_t = XYZEulertoHomogeneousMatrix(node_t.pose);
                voxel_map->Update(scan, T_t);
                visualization(graph, voxel_map);

                odom_t_1 = odom_t;
                u_t_1 = odom_t;
                sigma = initial_threshold;
                first_time = false;
            }
            u_t = odom_t;
            UpdateMotion(odom_t_1, odom_t, robot_pose, "omni");
            if(AddNode(u_t_1, u_t, min_trans, min_rot)) {
                ros::Time T1 = ros::Time::now();
                Eigen::Matrix4d initial_guess = XYZEulertoHomogeneousMatrix(robot_pose);
                Eigen::Matrix4d T_t = scanmatcher.Registration(icpscan,
                                                   voxel_map,
                                                   initial_guess,
                                                   3.0 * sigma,
                                                   sigma / 3.0);
                const Eigen::Matrix4d current_deviation = initial_guess.inverse() * T_t;                                  
                adaptive_threshold.UpdateModelDeviation(current_deviation);
                sigma = adaptive_threshold.ComputeThreshold();
                robot_pose.head(3) = T_t.block<3, 1>(0, 3);
                robot_pose[5] = atan2(T_t(1, 0), T_t(0, 0));
                ros::Time T2 = ros::Time::now();
                std::cout << "Time processing ICP: " << (T2-T1).toSec() << "s" << endl;

                node_t.pose = robot_pose;
                node_t.scan = scan;
                node_t.idx ++;
                graph.nodes.push_back(node_t);

                edge_t.i = node_t.idx - 1;
                edge_t.j = node_t.idx;
                graph.edges.push_back(edge_t);

                voxel_map->Update(scan, T_t);
                visualization(graph, voxel_map);
                u_t_1 = odom_t;
            }
            odom_t_1 = odom_t;
        }
        rate.sleep();
    }
}

void GraphBasedSlam::init() {
    if(!ros::param::get("~map_frame", map_frame))
        map_frame = "map";
    if(!ros::param::get("~base_frame", base_frame))
        base_frame = "vkbot";
    if(!ros::param::get("~odom_frame", odom_frame))
        odom_frame = "odom";
    if(!ros::param::get("~model_type", model_type))
        model_type = "omni";

    if(!ros::param::get("~frequency", frequency))
        frequency = 10;
    if(!ros::param::get("~anpha", anpha))
        anpha = 0.5;
    if(!ros::param::get("~beta", beta))
        beta = 1.5;
    if(!ros::param::get("~voxel_size", voxel_size))
        voxel_size = 0.3;
    if(!ros::param::get("~max_points_per_voxel", max_points_per_voxel))
        max_points_per_voxel = 20;
    if(!ros::param::get("~min_trans", min_trans))
        min_trans = 0.3;
    if(!ros::param::get("~min_rot", min_rot))
        min_rot = 0.3;
    if(!ros::param::get("~min_range", min_range))
        min_range = 0.05;
    if(!ros::param::get("~max_range", max_range))
        max_range = 30.0;
    if(!ros::param::get("~initial_threshold", initial_threshold))
        initial_threshold = 0.3;
    if(!ros::param::get("~min_motion", min_motion))
        min_motion = 0.1;
    received_data = false;

    marker_node.header.frame_id = "map";
    marker_node.header.stamp = ros::Time::now();
    marker_node.action = visualization_msgs::Marker::ADD;
    marker_node.type = visualization_msgs::Marker::SPHERE;
    marker_node.id = 0;
    marker_node.ns = "vk_3dslam_node";
    marker_node.scale.x = 0.1;
    marker_node.scale.y = 0.1;
    marker_node.scale.z = 0.1;
    marker_node.color.r = 1.0;
    marker_node.color.g = 0.0;
    marker_node.color.b = 0.0;
    marker_node.color.a = 1.0;
    marker_node.lifetime = ros::Duration(0);

    marker_edge.header.frame_id = "map";
    marker_edge.header.stamp = ros::Time::now();
    marker_edge.action = visualization_msgs::Marker::ADD;
    marker_edge.type = visualization_msgs::Marker::LINE_STRIP;
    marker_edge.id = 0;
    marker_edge.scale.x = 0.02;
    marker_edge.scale.y = 0.02;
    marker_edge.scale.z = 0.02;
    marker_edge.ns = "vk_3dslam_edge";
    marker_edge.color.r = 0.0;
    marker_edge.color.g = 0.0;
    marker_edge.color.b = 1.0;
    marker_edge.color.a = 1.0;
    marker_edge.lifetime = ros::Duration(0);
}

void GraphBasedSlam::dataCallback(const nav_msgs::Odometry& msg1, const sensor_msgs::PointCloud2& msg2) {
    odom = {msg1.pose.pose.position.x, 
            msg1.pose.pose.position.y,
            msg1.pose.pose.position.z,
            0.0,
            0.0,
            tf::getYaw(msg1.pose.pose.orientation)};
    
    Eigen::Vector6d tf_base_frame_to_lidar_frame = {-0.058, 0.0, 0.394, 0.0, 0.0, 0.0};
    Eigen::Matrix4d tf_base_frame_to_lidar_frame_matrix = XYZEulertoHomogeneousMatrix({tf_base_frame_to_lidar_frame});
    sensor_msgs::PointCloud pcl;
    sensor_msgs::convertPointCloud2ToPointCloud(msg2, pcl);
    
    std::vector<Eigen::Vector3d> points;
    Eigen::Vector3d point;
    for(auto& p : pcl.points) {
        point = {p.x, p.y, p.z};
        auto range = point.norm();
        if(range > min_range && range < max_range) {
            point = tf_base_frame_to_lidar_frame_matrix.block<3, 3>(0, 0) * point + tf_base_frame_to_lidar_frame_matrix.block<3, 1>(0, 3);
            points.push_back(point);
        }
    }

    scan = VoxelDownSample(points, anpha * voxel_size);
    icpscan = VoxelDownSample(scan, beta * voxel_size);
    received_data = true;
}

void GraphBasedSlam::visualization(Graph& graph, VoxelHashMap::Ptr voxel_map) {
    std::vector<Eigen::Vector3d> eigen_points = voxel_map->ExtractToPoints();
    sensor_msgs::PointCloud ros_map = voxel_map->EigenToPointCloudMsg(eigen_points, map_frame);
    map_pub.publish(ros_map);

    int num_nodes = graph.nodes.size();
    int num_edges = graph.edges.size();
    int id = 0;
    marker_graph.markers.clear();
    for(int i = 0; i < num_nodes; i++) {
        marker_node.id = id;
        marker_node.pose.position.x = graph.nodes[i].pose(0);
        marker_node.pose.position.y = graph.nodes[i].pose(1);
        marker_node.pose.position.z = graph.nodes[i].pose(2);
        marker_graph.markers.push_back(marker_node);
        id++;
    }
    
    for(int j = 0; j < num_edges; j++) {
        marker_edge.id = id;
        marker_edge.points.clear();
        geometry_msgs::Point p;

        int id_i = graph.edges[j].i;
        int id_j = graph.edges[j].j;

        p.x = graph.nodes[id_i].pose(0);
        p.y = graph.nodes[id_i].pose(1);
        p.z = graph.nodes[id_i].pose(2);
        marker_edge.points.push_back(p);

        p.x = graph.nodes[id_j].pose(0);
        p.y = graph.nodes[id_j].pose(1);
        p.z = graph.nodes[id_j].pose(2);
        marker_edge.points.push_back(p);

        marker_graph.markers.push_back(marker_edge);
        id++;
    }
    pose_graph_pub.publish(marker_graph);
}

}
