#pragma once
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include "vk_slam3d/voxelgrid.hpp"

namespace slam {

std::vector<Eigen::Vector3d> VoxelDownSample(const std::vector<Eigen::Vector3d>& points,
                                             double voxel_size);
std::vector<Eigen::Vector3d> PreProcessing(const std::vector<Eigen::Vector3d>& points,
                                           double min_range,
                                           double max_range);

}