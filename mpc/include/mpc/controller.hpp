#pragma once
#include "mpc/mpc.hpp"

void obstaclesCallback(const visualization_msgs::MarkerArray& msg) {
    Obstacle p;
    Point vert;
    set_obs.clear();
    for(int i = 0; i < msg.markers.size(); i++) {
        p.vertices.clear();
        for(int j = 0; j < msg.markers[i].points.size(); j++) {
            vert.x = msg.markers[i].points[j].x*cm_m;
            vert.y = msg.markers[i].points[j].y*cm_m;
            p.vertices.push_back(vert);
        }
        set_obs.push_back(p);
    }
    N_obstacles = set_obs.size();
    obstacles_ = true;
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
	path.header.frame_id = map_frame;
	path.header.stamp = ros::Time::now();
	p.pose.position.x = q.x/cm_m;
	p.pose.position.y = q.y/cm_m;
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
		p.pose.position.x = q.x/cm_m;
		p.pose.position.y = q.y/cm_m;
		path.poses.push_back(p);	
	}
	tra_pub.publish(path);;
	path_ = false;
    trajectory_ = true;
    control_ = true;
    t = 0;
}

bool compare_obstacles(Obstacle p1, Obstacle p2) {
    double d1 = pow(lz_pose.x - p1.x, 2) + pow(lz_pose.y - p1.y, 2);
    double d2 = pow(lz_pose.x - p2.x, 2) + pow(lz_pose.y - p2.y, 2);
    return (d1 < d2);   
}

void get_obstacles(Obstacle& p) {
    double a_so = 0;
    double mx = 0, my = 0;
    double m;
    if(p.vertices.size() > 2) {
        p.vertices.push_back(p.vertices[0]);
        for(int i = 0; i < p.vertices.size()-1; i++) {
            m = p.vertices[i].x*p.vertices[i+1].y - p.vertices[i+1].x*p.vertices[i].y;
            a_so = a_so + m/2;
            mx += (p.vertices[i].x + p.vertices[i+1].x)*m;
            my += (p.vertices[i].y + p.vertices[i+1].y)*m;
        }
        if(fabs(a_so) != 0) {
            p.x = mx/(6*a_so);
            p.y = my/(6*a_so);
            p.vertices.pop_back();
            double d_max = sqrt(pow(p.x - p.vertices[0].x,2) + pow(p.y - p.vertices[0].y,2));
            for(int i = 0; i < p.vertices.size(); i++) {
                double d = sqrt(pow(p.x - p.vertices[i].x,2) + pow(p.y - p.vertices[i].y,2));
                if(d > d_max) {
                    d_max = d;
                }
            }
            p.radius = d_max;
             if(d_max > 0.1) {
                 p.radius = d_max;
             }else {
                 p.radius = 0.1;
             }
        }else {
            p.vertices.pop_back();
            p.x = (p.vertices[1].x + p.vertices[2].x)/2;
            p.y = (p.vertices[1].y + p.vertices[2].y)/2;
            p.radius = 0.5*sqrt(pow(p.vertices[1].x - p.vertices[2].x,2) + pow(p.vertices[1].y - p.vertices[2].y,2));
             if(p.radius < 0.1) {
                 p.radius = 0.1;
             }
        }
    }else {
        ROS_WARN("Numpoint of the polygon is 2!");
        p.x = (p.vertices[0].x + p.vertices[1].x)/2;
        p.y = (p.vertices[0].y + p.vertices[1].y)/2;
        p.radius = 0.5*sqrt(pow(p.vertices[0].x - p.vertices[1].x,2) + pow(p.vertices[0].y - p.vertices[1].y,2));
    }
    p.v = 0.0;
    p.theta = 0.0;
    // if(p.vertices.size() > 3) {
    //     for(int i = 0; i < p.vertices.size()-1; i++) {
    //         m = p.vertices[i].x*p.vertices[i+1].y - p.vertices[i+1].x*p.vertices[i].y;
    //         a_so = a_so + m/2;
    //         mx += (p.vertices[i].x + p.vertices[i+1].x)*m;
    //         my += (p.vertices[i].y + p.vertices[i+1].y)*m;
    //     }
    //     if(fabs(a_so) != 0) {
    //         p.x = mx/(6*a_so);
    //         p.y = my/(6*a_so);
    //         p.vertices.pop_back();
    //         double d_max = sqrt(pow(p.x - p.vertices[0].x,2) + pow(p.y - p.vertices[0].y,2));
    //         for(int i = 0; i < p.vertices.size(); i++) {
    //             double d = sqrt(pow(p.x - p.vertices[i].x,2) + pow(p.y - p.vertices[i].y,2));
    //             if(d > d_max) {
    //                 d_max = d;
    //             }
    //         }
    //         p.radius = d_max;
    //         // if(d_max > 0.1) {
    //         //     p.radius = d_max;
    //         // }else {
    //         //     p.radius = 0.1;
    //         // }
    //     }else {
    //         ROS_WARN("Area is 0");
    //     }
    // }else {
    //     p.vertices.pop_back();
    //     p.x = (p.vertices[0].x + p.vertices[1].x)/2;
    //     p.y = (p.vertices[0].y + p.vertices[1].y)/2;
    //     p.radius = 0.5*sqrt(pow(p.vertices[0].x - p.vertices[1].x,2) + pow(p.vertices[0].y - p.vertices[1].y,2));
    //     // if(p.radius < 0.1) {
    //     //     p.radius = 0.1;
    //     // }
    // }
}

void get_obstacles_nearest(vector<Obstacle>& set_obs_, vector<Obstacle>& set_obs_nearest_) {
    for(int i = 0; i < set_obs_.size(); i++) {
        get_obstacles(set_obs_[i]);
    }
    sort(set_obs_.begin(), set_obs_.end(), compare_obstacles);
    set_obs_nearest_.clear();
    if(N_obstacles > N_maxobstacles) {
        for(int i = 0; i < N_maxobstacles; i++) {
            set_obs_nearest_.push_back(set_obs_[i]);
        }
    }else {
        for(int i = 0; i < N_obstacles; i++) {
            set_obs_nearest_.push_back(set_obs_[i]);
        }
        for(int i = 0; i < N_maxobstacles - N_obstacles; i++) {
            set_obs_nearest_.push_back(set_obs_[0]);
        }
    }  
}