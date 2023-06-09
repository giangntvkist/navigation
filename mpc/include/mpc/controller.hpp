#pragma once
#include "mpc/mpc.hpp"

void obstaclesCallback(const costmap_converter::ObstacleArrayMsg& msg) {
    obstacles_ = true;
    N_maxobstacles = 5;
}

void trajectory_publisher() {
    numerical_velocity_profile(0, 0);
    nav_msgs::Path path;
	path.poses.clear();
	State q;
    switch (trajectory_type) {
        case bezier_:
            {
                q = bezier_trajectory(0);
                break;
            }
        case scurve_:
            {
                q = scurve_trajectory(0);
                break;
            }
        default:
            {
                ROS_ERROR("Trajectory type incorrect!");
            }
    }
	geometry_msgs::PoseStamped p;
	path.header.frame_id = "map_frame";
	path.header.stamp = ros::Time::now();
	p.pose.position.x = int(q.x/cm_m);
	p.pose.position.y = int(q.y/cm_m);
	path.poses.push_back(p);  
    for(int i = 1; i < T[M]/0.01; i++){	
		double dt = i*0.01;
        switch (trajectory_type) {
        case bezier_:
            {
                q = bezier_trajectory(dt);
                break;
            }
        case scurve_:
            {
                q = scurve_trajectory(dt);
                break;
            }
        default:
            {
                ROS_ERROR("Trajectory type incorrect!");
            }
    }
		p.pose.position.x = int(q.x/cm_m);
		p.pose.position.y = int(q.y/cm_m);
		path.poses.push_back(p);	
	}
	tra_pub.publish(path);;
	path_ = false;
    trajectory_ = true;
    control_ = true;
    t = 0;
}
