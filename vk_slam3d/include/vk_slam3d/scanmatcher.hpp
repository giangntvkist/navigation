#pragma once
#include <ros/ros.h>
#include <tuple>
#include <eigen3/Eigen/Core>
#include <thread>
#include <mutex>
#include <boost/thread.hpp>

#include "vk_slam3d/voxelhashmap.hpp"

namespace slam {

inline double square(double x) { return x * x; }

struct Correspondence {
    Eigen::Vector3d p;
    Eigen::Vector3d q;
    double dist2;
};

class ScanMatcher {
    private:
        void TransformPoints(const Eigen::Matrix4d& T,
                             std::vector<Eigen::Vector3d>& points);

        void GetCorrespondences(const std::vector<Eigen::Vector3d>& source,
                                const VoxelHashMap::Ptr voxel_map,
                                double max_correspondance_distance,
                                std::vector<Correspondence>& Corres);

        void BuildLinearSystem(const std::vector<Correspondence>& Corres,
                               double kernel,
                               Eigen::Matrix6d& JTJ,
                               Eigen::Vector6d& JTr);
        int max_iterations = 500;
        double epsilon = 1e-4;
    public:
        Eigen::Matrix4d Registration(const std::vector<Eigen::Vector3d>& currentScan,
                                     const VoxelHashMap::Ptr voxel_map,
                                     const Eigen::Matrix4d& initial_guess,
                                     double max_correspondence_distance,
                                     double kernel);
        
};

}