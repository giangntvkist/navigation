#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/CameraInfo.h>

#include <math.h>
#include <vector>

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

#ifdef DEBUG
uint16_t Byte2Int(uint8_t byHigh, uint8_t byLow) {
    return ((uint16_t)byHigh << 8 | byLow);
}
#endif

cv_bridge::CvImageConstPtr cv_ptr;
void depthimageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    cv_ptr = cv_bridge::toCvShare(msg);
    #ifdef DEBUG
    depth_image.height = cv_ptr->image.rows;
    depth_image.width = cv_ptr->image.cols;
    distanceVal = cv_ptr->image.at<uint16_t>(i, j)*0.001;
    /* distanceVal = 0.001*Byte2Int(msg->data[2*i*depth_image.width+2*j+1], msg->data[2*i*depth_image.width+2*j]) */
    #endif
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

    T[3][0] = 0;
    T[3][1] = 0;
    T[3][2] = 0;
    T[3][3] = 1;
}

void depthImage_LaserScan(cv_bridge::CvImageConstPtr& cv_ptr, cameraInfo& camera_info, vector<scan_t>& laser_scan) {
    int num_columns = cv_ptr->image.cols;
    double fx, fy, cx, cy;
    fx = camera_info.intrinsic_matrix[0][0];
    fy = camera_info.intrinsic_matrix[1][1];
    cx = camera_info.intrinsic_matrix[0][2];
    cy = camera_info.intrinsic_matrix[1][2];

    laser_scan.clear();
    int num_pixels = 5; /* number of pixels inside each group */
    for(int j = 0; j < num_columns; j++) {
        double phi = -atan((j - cx)/fx);
        double r = myinf;
        int num_groups = cv_ptr->image.rows / num_pixels;
        scan_t scan;
        for(int k = 0; k < num_groups; k++) {
            /* pixel p with  the smallest distance */
            double d_min = myinf;
            pixel_t p;
            for(int i = num_pixels*k; i < num_pixels*k + num_pixels; i++) {
                if(cv_ptr->image.at<uint16_t>(i, j) < d_min) {
                    d_min = cv_ptr->image.at<uint16_t>(i, j);
                    p.i = i;
                    p.j = j;
                }
            }

            /* Transform pixel p into Ks coordinate */
            point3d_t pt_s;
            pt_s.x = (cv_ptr->image.at<uint16_t>(p.i, p.j)-4.2)*0.001; /* x_s */ /* -4.2 mm thong so khoang cach thuc te den ong kinh */
            pt_s.y = -(p.j - cx) * pt_s.x / fx; /* y_s */
            pt_s.z = -(p.i - cy) * pt_s.x / fy; /* z_s */
            double r_p = sqrt(pow(pt_s.x, 2) + pow(pt_s.y, 2)); /* r(p) */
            bool br = (r_p >= range_min && r_p <= range_max); /* if r(p) thuoc [r_min, r_max] return True else return False */

            /* Transform z_s(p) intto K0 coordinate (base_link) */
            double tf_[4][4];
            transform(camera_info, tf_);
            point3d_t pt_rb;
            pt_rb.x = tf_[0][0]*pt_s.x + tf_[0][1]*pt_s.y + tf_[0][2]*pt_s.z + tf_[0][3];
            pt_rb.y = tf_[1][0]*pt_s.x + tf_[1][1]*pt_s.y + tf_[1][2]*pt_s.z + tf_[1][3];
            pt_rb.z = tf_[2][0]*pt_s.x + tf_[2][1]*pt_s.y + tf_[2][2]*pt_s.z + tf_[2][3];
            bool bz = (pt_rb.z >= z_min && pt_rb.z <= z_max);
        
            if(br && bz && r_p < r) {
                r = r_p;
                scan.point[0] = pt_rb.x;
                scan.point[1] = pt_rb.y;
                scan.point[2] = pt_rb.z;

                scan.range = r;
                scan.angle = phi;
            }
        }
        laser_scan.push_back(scan);
    }
}

int main(int argc,char **argv) {
    ros::init(argc, argv, "depthimage_to_laserscan");
    ros::NodeHandle nh;

    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud>("pcl_camera_01", 10);

    ros::Subscriber depthimage_sub = nh.subscribe("/camera/aligned_depth_to_color/image_raw", 10, depthimageCallback);
    ros::Subscriber camerainfo_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info", 1, camerainfoCallback);

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
            depthImage_LaserScan(cv_ptr, camera_info, laser_scan);
            int num_points = laser_scan.size();
            pcl_laserscan.header.frame_id = "base_link";
            pcl_laserscan.points.clear();
            geometry_msgs::Point32 point;
            for(int i = 0 ; i < num_points; i++){
                point.x = laser_scan[i].point[0];
                point.y = laser_scan[i].point[1];
                // point.z = laser_scan[i].point[2];
                point.z = 0.0;
                pcl_laserscan.points.push_back(point);
            }
            pcl_pub.publish(pcl_laserscan);
        }
        rate.sleep();
    }
}