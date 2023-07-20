#include "vk_slam/vk_slam.hpp"
#include "vk_slam/observation.hpp"
#include "vk_slam/optimization.hpp"
#include "vk_slam/scan_matching.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "vk_slam");
    ROS_INFO("Running vk_slam node!");
    ros::NodeHandle nh;

    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 10);
    pose_graph_pub = nh.advertise<nav_msgs::Path>("pose_graph", 10);

    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 10);
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_scan_sub(nh, "scan1", 10);
    typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10),odom_sub, laser_scan_sub);
    sync.registerCallback(dataCallback);

    if(!ros::param::get("~model_type", model_type)) model_type = omni;
    if(!ros::param::get("~range_min", range_min)) range_min = 0.05;
    if(!ros::param::get("~range_max", range_max)) range_max = 25.0;
    if(!ros::param::get("~angle_min", angle_min)) angle_min = -M_PI/2;
    if(!ros::param::get("~angle_max", angle_max)) angle_max = M_PI/2;
    if(!ros::param::get("~angle_increment", angle_increment)) angle_increment = M_PI/(3*180);

    if(!ros::param::get("~laser_pose_x", laser_pose_x)) laser_pose_x = 0.289;
    if(!ros::param::get("~laser_pose_y", laser_pose_y)) laser_pose_y = -0.0;
    if(!ros::param::get("~laser_pose_theta", laser_pose_theta)) laser_pose_theta = 0.0;
    if(!ros::param::get("~throttle_scan", throttle_scan)) throttle_scan = 5;
    if(!ros::param::get("~inverted_laser", inverted_laser)) inverted_laser = true;

    if(!ros::param::get("~init_pose_x", init_pose_x)) init_pose_x = 0.0;
    if(!ros::param::get("~init_pose_y", init_pose_y)) init_pose_y = 0.0;
    if(!ros::param::get("~init_pose_theta", init_pose_theta)) init_pose_theta = 0.0;

    if(!ros::param::get("~max_inter", max_inter)) max_inter = 100;
    if(!ros::param::get("~converged_graph", converged_graph)) converged_graph = 1e-3;
    if(!ros::param::get("~map_update_interval", map_update_interval)) map_update_interval = 10;

    if(!ros::param::get("~min_trans", min_trans)) min_trans = 0.2;
    if(!ros::param::get("~min_rot", min_rot)) min_rot = 0.2;
    if(!ros::param::get("~dist_threshold", dist_threshold)) dist_threshold = 1.0;

    if(!ros::param::get("~map_width", map_width)) map_width = 500;
    if(!ros::param::get("~map_height", map_height)) map_height = 500;
    if(!ros::param::get("~map_resolution", map_resolution)) map_resolution = 0.02;

    if(!ros::param::get("~base_frame", base_frame)) base_frame = "base_link";
    if(!ros::param::get("~map_frame", map_frame)) map_frame = "vk_map";

    nav_msgs::OccupancyGrid map;
    vector<double> log_map_t_;
    nav_msgs::Path pose_graph;

    sl_graph_t graph_t;
    sl_vector_t odom_t, odom_t_1;
    sl_vector_t u_t[2];
    sl_vector_t u_t_[2];
    sl_node_t node_current;
    sl_vector_t pose_robot_t;
    int num_nodes = 0;

    init_slam(log_map_t_, map, pose_graph);
    ros::Time T1, T2;
    ros::Rate rate(map_update_interval);
    while(ros::ok()) {
        ros::spinOnce();
        if(data_) {
            odom_t.v[0] = odom.pose.pose.position.x;
            odom_t.v[1] = odom.pose.pose.position.y;
            odom_t.v[2] = tf::getYaw(odom.pose.pose.orientation);
            if(first_time) {
                pose_robot_t.v[0] = init_pose_x;
                pose_robot_t.v[1] = init_pose_y;
                pose_robot_t.v[2] = init_pose_theta;

                node_current.pose = pose_robot_t;
                node_current.scan = scan_t;
                graph_t.set_node_t.push_back(node_current);
                num_nodes += 1;

                odom_t_1 = odom_t;
                u_t[0] = odom_t;
                first_time = false;
            }
            u_t[1] = odom_t;
            u_t_[0] = odom_t_1;
            u_t_[1] = odom_t;
            update_motion(u_t_, pose_robot_t);
            if(update_node(u_t)) {
                node_current.pose = pose_robot_t;
                node_current.scan = scan_t;
                graph_t.set_node_t.push_back(node_current);
                num_nodes += 1;
                vanilla_ICP(graph_t.set_node_t[num_nodes-2], graph_t.set_node_t[num_nodes-1]);
                u_t[0] = odom_t;
            }
            mapping(graph_t, log_map_t_, map, pose_graph);
            odom_t_1 = odom_t;
        }
        rate.sleep();
    }
    return 0;
}