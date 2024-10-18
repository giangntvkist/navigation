#include "vk_slam3d/voxelhashmap.hpp"
namespace slam {

void VoxelHashMap::AddPoints(const std::vector<Eigen::Vector3d>& points) {
    for(auto& point : points) {
        Eigen::Vector3i voxel = (point / voxel_size).cast<int>();
        map.insert(point, voxel, max_points_per_voxel);
    }
}

void VoxelHashMap::Update(std::vector<Eigen::Vector3d>& points,
                          Eigen::Matrix4d& T_t) {
    std::vector<Eigen::Vector3d> points_transformed;
    for(auto& point : points) {
        Eigen::Vector3d point_transformed = T_t.block<3, 3>(0, 0) * point + T_t.block<3, 1>(0, 3);
        points_transformed.push_back(point_transformed);
    }
    AddPoints(points_transformed);
}

std::vector<Eigen::Vector3d> VoxelHashMap::ExtractToPoints() {
    std::vector<Eigen::Vector3d> points;
    Voxel* temp;
    for(int i = 0; i < map.gethashsize(); i++) {
        temp = map.hash_table[i];
        while(temp != NULL) {
            for(auto& point : temp->points) { points.push_back(point); }
            temp = temp->next;
        }
    }
    return points;
}

sensor_msgs::PointCloud VoxelHashMap::EigenToPointCloudMsg(const std::vector<Eigen::Vector3d>& points,
                                                           std::string frame) {
    sensor_msgs::PointCloud pointclouds;
    pointclouds.header.frame_id = frame;
    pointclouds.header.stamp = ros::Time::now();
    for(auto& p : points) {
        geometry_msgs::Point32 point_msg;
        point_msg.x = p[0];
        point_msg.y = p[1];
        point_msg.z = p[2];
        pointclouds.points.push_back(point_msg);
    }
    return pointclouds;
}

}