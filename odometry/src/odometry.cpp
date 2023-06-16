#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <math.h>
#include <sstream>

using namespace std;

/* Wheels velocity */
double V_right, V_left; // [m/s]

/* Robot model parameters */
double R; // Wheel radius [m]
double L; // Distance of two wheels [m]

/* Robot velocity */
double V; // linear velocity [m/s]
double W; // angular velocity [rad/s]

/* Robot position in global coordinate */
double x; // x position [m]
double y; // y position [m]
double theta; // orientation [rad]

bool data;
double publish_frequency;
nav_msgs::Odometry odom_;
string odom_frame, robot_frame;

void velocityCallback(const std_msgs::Float64MultiArray& msg) {
    V_right = msg.data[0];
    V_left = msg.data[1];
    data = true;
}

void normalize(double& theta_) {
    theta_ = atan2(sin(theta_), cos(theta_));
}

int main(int argc, char **argv) {
    ros::init(argc,argv,"odometry");
    ROS_INFO("Running odometry node!");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom",10);
    ros::Subscriber vel_sub = nh.subscribe("wheels_speed",10,velocityCallback);

    if(!ros::param::get("~R", R)) R = 0.0625;
    if(!ros::param::get("~L", L)) L = 0.5;

    if(!ros::param::get("~publish_frequency", publish_frequency)) publish_frequency = 100;
    if(!ros::param::get("~odom_frame", odom_frame)) odom_frame = "odom";
    if(!ros::param::get("~robot_frame", robot_frame)) robot_frame = "base_link";
    
    x = 0.0;
    y = 0.0;
    theta = 0.0;
    ros::Time current_time, last_time;

    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate rate(publish_frequency);
    while(nh.ok()) {
        ros::spinOnce();
            current_time = ros::Time::now();
            double dt = (current_time - last_time).toSec();
            V = 0.5*(V_right + V_left);
            W = 0.5*(V_right - V_left);

            x += V*cos(theta)*dt;
            y += V*sin(theta)*dt;
            theta += W*dt;

            odom_.header.stamp = current_time;
            odom_.header.frame_id = odom_frame;
            odom_.child_frame_id = robot_frame;
  
            odom_.pose.pose.position.x = x;
            odom_.pose.pose.position.y = y;
            odom_.pose.pose.position.z = 0.0;
            odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);;
  
        
            odom_.twist.twist.linear.x = V*cos(theta);
            odom_.twist.twist.linear.y = V*sin(theta);
            odom_.twist.twist.angular.z = W;
            odom_pub.publish(odom_);

            normalize(theta);
            last_time = current_time;
        rate.sleep();
    }
    return 0;
}




