#pragma once
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
using namespace std;
namespace slam {

struct Voxel {
    Eigen::Vector3i position;
    std::vector<Eigen::Vector3d> points;
    Voxel* next;
};

class VoxelGrid {
    private:
        static const int hash_size = (1 << 20 - 1);
        int hashfunction(const Eigen::Vector3i& voxel);

    public:
        void insert(const Eigen::Vector3d& point,
                    const Eigen::Vector3i& voxel);
        void insert(const Eigen::Vector3d& point,
                    const Eigen::Vector3i& voxel, 
                    int max_points_per_voxel);
        
        std::vector<Eigen::Vector3d> getpoints(const Eigen::Vector3i& voxel);
        bool empty() const { for(auto& pt : hash_table) { if(pt != NULL) { return false; }} return true; }
        Voxel* hash_table[hash_size];

        inline int gethashsize() { return hash_size; }
};

}