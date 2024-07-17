#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <nav_msgs/OccupancyGrid.h>
#include <math.h>

using namespace std;

#define myinf 1e6

struct scan_t {
    double point[3];
    double range;
    double angle;
};

struct cameraInfo {
    double intrinsic_matrix[3][3]; /* unit is pixel */
    double trans[3];
    double rot[3];
};

struct pixel_t {
    int i;
    int j;
};

struct point3d_t {
    double x;
    double y;
    double z;
};

double publish_frequency;
double z_min, z_max;
double range_min, range_max;

bool depth_image_data = false;

sensor_msgs::PointCloud pcl_depthcamera;
void pointcloud2Callback(const sensor_msgs::PointCloud2& msg) {
    sensor_msgs::convertPointCloud2ToPointCloud(msg, pcl_depthcamera);
    depth_image_data = true;
}

cameraInfo camera_info;
void camerainfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg) {
    camera_info.intrinsic_matrix[0][0] = msg->K[0];
    camera_info.intrinsic_matrix[0][2] = msg->K[2];
    camera_info.intrinsic_matrix[1][1] = msg->K[4];
    camera_info.intrinsic_matrix[1][2] = msg->K[5];

    camera_info.trans[0] = 0.0;
    camera_info.trans[1] = 0.0;
    camera_info.trans[2] = 0.0;

    camera_info.rot[0] = 0.0;
    camera_info.rot[1] = 0.0;
    camera_info.rot[2] = 0.0;
}

void transform(cameraInfo& camera_info, double (&T)[4][4]) {
    double rotX = camera_info.rot[0];
    double rotY = camera_info.rot[1];
    double rotZ = camera_info.rot[2];

    T[0][0] = cos(rotZ)*cos(rotY);
    T[0][1] = cos(rotZ)*sin(rotY)*sin(rotX) - sin(rotZ)*cos(rotX);
    T[0][2] = cos(rotZ)*sin(rotY)*cos(rotX) + sin(rotZ)*sin(rotX);
    T[0][3] = camera_info.trans[0];

    T[1][0] = sin(rotZ)*cos(rotY);
    T[1][1] = sin(rotZ)*sin(rotY)*sin(rotX) + cos(rotZ)*cos(rotX);
    T[1][2] = sin(rotZ)*sin(rotY)*cos(rotX) - cos(rotZ)*sin(rotX);
    T[1][3] = camera_info.trans[1];

    T[2][0] = -sin(rotY);
    T[2][1] = cos(rotY)*sin(rotX);
    T[2][2] = cos(rotY)*cos(rotX);
    T[2][3] = camera_info.trans[2];

    T[3][0] = 0.0;
    T[3][1] = 0.0;
    T[3][2] = 0.0;
    T[3][3] = 1.0;
}

void depthImage_LaserScan(sensor_msgs::PointCloud& pcl_depthcamera, cameraInfo& camera_info, vector<scan_t>& laser_scan) {
    int num_columns = pcl_depthcamera.channels[].values[];
    int num_rows = pcl_depthcamera.channels[].values[];
    laser_scan.clear();
    int num_pixels = 5; /* number of pixels inside each group */
    for(int j = 0; j < num_columns; j++) {
        /*............... */
    }
}


int main(int argc,char **argv) {
    ros::init(argc, argv, "depthimage_to_laserscan");
    ros::NodeHandle nh;

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud>("pcl_camera_01", 10);

    ros::Subscriber pcl2_sub = nh.subscribe("", 10, pointcloud2Callback);
    ros::Subscriber camerainfo_sub = nh.subscribe("", 1, camerainfoCallback);

    if(!ros::param::get("~publish_frequency", publish_frequency)) publish_frequency = 10;
    if(!ros::param::get("~z_min", z_min)) z_min = -0.0;
    if(!ros::param::get("~z_max", z_max)) z_max = 0.5;

    if(!ros::param::get("~range_min", range_min)) range_min = 0.05;
    if(!ros::param::get("~range_max", range_max)) range_max = 3.0;
    
    sensor_msgs::PointCloud pcl_laserscan;
    vector<scan_t> laser_scan;

    ros::Time current_time;
    ros::Rate rate(publish_frequency);
    while(ros::ok()) {
        ros::spinOnce();
        current_time = ros::Time::now();
        if(depth_image_data) {
            /*............... */
            pcl_pub.publish(pcl_laserscan);
        }
        rate.sleep();
    }
}