#include "vk_slam/vk_slam.hpp"
#include "vk_slam/optimization.cpp"
int main(int argc, char **argv) {
    ros::init(argc, argv, "vk_slam");
    ROS_INFO("Running vk_slam node!");
    ros::NodeHandle nh;

    ros::Publisher path_real_pub = nh.advertise<nav_msgs::Path>("path_real", 100);
    ros::Publisher path_old_pub = nh.advertise<nav_msgs::Path>("path_old", 100);
    ros::Publisher path_new_pub = nh.advertise<nav_msgs::Path>("path_new", 100);

    pose_graph_test(graph_t);
    sl_graph_t graph_t_old = graph_t;
    optimization(graph_t);
    ros::Rate rate(map_update_interval);
    while(ros::ok()) {
        ros::spinOnce();
        nav_msgs::Path path_2;
        geometry_msgs::PoseStamped p;
        path_2.header.frame_id = "map";
        path_2.header.stamp = ros::Time::now();
        path_2.poses.clear();
        for(int i = 0; i < graph_t_old.set_node_t.size(); i++) {
            p.pose.position.x = graph_t_old.set_node_t[i].pose.v[0];
            p.pose.position.y = graph_t_old.set_node_t[i].pose.v[1];
            p.pose.position.z = 0.0;
            path_2.poses.push_back(p);
        }
        path_old_pub.publish(path_2);

        nav_msgs::Path path_1;
        path_1.header.frame_id = "map";
        path_1.header.stamp = ros::Time::now();
        path_1.poses.clear();
        for(int i = 0; i < graph_t.set_node_t.size(); i++) {
            p.pose.position.x = graph_t.set_node_t[i].pose.v[0];
            p.pose.position.y = graph_t.set_node_t[i].pose.v[1];
            p.pose.position.z = 0.0;
            path_1.poses.push_back(p);
        }
        path_new_pub.publish(path_1);
        rate.sleep();
    }
    return 0;
}