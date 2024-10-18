#include "vk_slam3d/preprocessing.hpp"

namespace slam {

std::vector<Eigen::Vector3d> VoxelDownSample(const std::vector<Eigen::Vector3d>& points, double voxel_size) {
    VoxelGrid* voxelgrid = new VoxelGrid();
    for(auto& point : points) {
        Eigen::Vector3i voxel = (point / voxel_size).cast<int>();
        voxelgrid->insert(point, voxel);
    }
    std::vector<Eigen::Vector3d> points_downsampled;
    Voxel* temp;
    for(int i = 0; i < voxelgrid->gethashsize(); i++) {
        temp = voxelgrid->hash_table[i];
        while(temp != NULL) {
            points_downsampled.push_back(temp->points[0]);
            temp = temp->next;
        }
    }
    delete [] voxelgrid;
    return points_downsampled;
}

std::vector<Eigen::Vector3d> PreProcessing(const std::vector<Eigen::Vector3d>& points,
                                           double min_range,
                                           double max_range) {
    std::vector<Eigen::Vector3d> inliers;
    for(auto& pt : points) {
        double range = pt.norm();
        if(range > min_range && range < max_range) inliers.push_back(pt);
    }
    return inliers;
}

}