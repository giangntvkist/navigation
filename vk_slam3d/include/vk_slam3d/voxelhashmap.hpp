#pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <eigen3/Eigen/Core>
#include "vk_slam3d/voxelgrid.hpp"
#include "vk_slam3d/transformhomogeneous.hpp"

namespace slam {

class VoxelHashMap {
    private:
        VoxelGrid map;
    public:
        VoxelHashMap(double voxel_size_, int max_points_per_voxel_)
            : voxel_size(voxel_size_),
              max_points_per_voxel(max_points_per_voxel_) {};
              
        inline bool Empty() const { return map.empty(); }
        void AddPoints(const std::vector<Eigen::Vector3d>& points);
        void Update(std::vector<Eigen::Vector3d>& points, Eigen::Matrix4d& T_t);
        std::vector<Eigen::Vector3d> GetPoints(const Eigen::Vector3i& voxel) { return map.getpoints(voxel); }
        std::vector<Eigen::Vector3d> ExtractToPoints();
        sensor_msgs::PointCloud EigenToPointCloudMsg(const std::vector<Eigen::Vector3d>& points,
                                                     std::string frame);

        double voxel_size;
        int max_points_per_voxel;

        typedef boost::shared_ptr<VoxelHashMap> Ptr;
        typedef boost::shared_ptr<const VoxelHashMap> ConstPtr;
};

}