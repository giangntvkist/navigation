#pragma once
#include "vk_slam/vk_slam.hpp"

double normalize(double z) {
    return atan2(sin(z), cos(z));
}

double angle_diff(double a, double b) {
    double d1, d2;
    a = normalize(a);
    b = normalize(b);
    d1 = a - b;
    d2 = 2*M_PI - fabs(d1);
    if(d1 > 0) d2 *= -1.0;
    if(fabs(d1) < fabs(d2)) {
        return d1;
    }else {
        return d2;
    }
}

bool scan_valid(double z) {
    return z < range_max && z >= range_min;
}
void dataCallback(const nav_msgs::Odometry& msg, const sensor_msgs::LaserScan& scan) {
    scan_t.ranges.clear();
    sl_vector_t ray_i;
    float inv_scan;
    if(inverted_laser) {
        inv_scan = -1.0;
    }else {
        inv_scan = 1.0;
    }
    for(int i = 0; i < scan.ranges.size(); i += throttle_scan) {
        if(scan_valid(scan.ranges[i])) {
            ray_i.v[0] = scan.ranges[i];
            ray_i.v[1] = inv_scan*(i*throttle_scan * angle_increment - angle_max);
            scan_t.ranges.push_back(ray_i);
        } 
    }
    odom_t.v[0] = msg.pose.pose.position.x;
    odom_t.v[1] = msg.pose.pose.position.y;
    odom_t.v[2] = tf::getYaw(msg.pose.pose.orientation);
    data_ = true;
}