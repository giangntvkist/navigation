#include "vk_slam3d/scanmatcher.hpp"

namespace slam {

void ScanMatcher::TransformPoints(const Eigen::Matrix4d& T, std::vector<Eigen::Vector3d>& points) {
    for(auto& point : points) {
        point = T.block<3, 3>(0, 0) * point + T.block<3, 1>(0, 3);
    }
}

void ScanMatcher::GetCorrespondences(const std::vector<Eigen::Vector3d>& source,
                                     const VoxelHashMap::Ptr voxel_map,
                                     double max_correspondance_distance,
                                     std::vector<Correspondence>& Corres) {
    Corres.clear();
    Eigen::Vector3i voxel;
    std::vector<Eigen::Vector3d> neighbors;
    int wsize = 1;
    Correspondence corre;
    for(auto& point : source) {
        auto kx = static_cast<int>(point[0] / voxel_map->voxel_size);
        auto ky = static_cast<int>(point[1] / voxel_map->voxel_size);
        auto kz = static_cast<int>(point[2] / voxel_map->voxel_size);

        Eigen::Vector3d closest_point;
        double closest_dist2 = std::numeric_limits<double>::max();
        std::vector<Eigen::Vector3i> voxels;
        for(int i = kx - wsize; i <= kx + wsize; i++) {
            for(int j = ky - wsize; j <= ky + wsize; j++) {
                for(int k = kz - wsize; k <= kz + wsize; k++) {
                    voxel = {i, j, k};
                    neighbors.clear();
                    neighbors = voxel_map->GetPoints(voxel);
                    for(const auto& point_neighbor : neighbors) {
                        double dist2 = (point_neighbor - point).squaredNorm();
                        if(dist2 < closest_dist2) {
                            closest_point = point_neighbor;
                            closest_dist2 = dist2;
                        }
                    }
                }
            }
        }
        if(sqrt(closest_dist2) < max_correspondance_distance) {
            corre.p = point;
            corre.q = closest_point;
            corre.dist2 = closest_dist2;
            Corres.push_back(corre);
        }
    }
}

void ScanMatcher::BuildLinearSystem(const std::vector<Correspondence>& Corres,
                                    double kernel,
                                    Eigen::Matrix6d& JTJ,
                                    Eigen::Vector6d& JTr) {
    JTJ.setZero();
    JTr.setZero();                          
    Eigen::Matrix3x6d J_r;
    Eigen::Vector3d residual;
    int N = Corres.size();
    for(int i = 0; i < N; i++) {
        residual = Corres[i].p - Corres[i].q;
        J_r.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        J_r.block<3, 3>(0, 3) = -1.0 * SO3intolie(Corres[i].p);
        double w = square(kernel)/ square(kernel + residual.squaredNorm());
        JTJ += J_r.transpose() * w * J_r;
        JTr += J_r.transpose() * w * residual;
    }
}

Eigen::Matrix4d ScanMatcher::Registration(const std::vector<Eigen::Vector3d>& currentScan,
                                          const VoxelHashMap::Ptr voxel_map,
                                          const Eigen::Matrix4d& initial_guess,
                                          double max_correspondence_distance,
                                          double kernel) {
    std::vector<Eigen::Vector3d> source = currentScan;
    TransformPoints(initial_guess, source);
    Eigen::Matrix4d T_icp = Eigen::Matrix4d::Identity();
    std::vector<Correspondence> Corres;
    Eigen::Matrix6d JTJ;
    Eigen::Vector6d JTr;
    for(int i = 0; i < max_iterations; i++) {
        GetCorrespondences(source, voxel_map, max_correspondence_distance, Corres);
        BuildLinearSystem(Corres, kernel, JTJ, JTr);
        Eigen::Vector6d dx = JTJ.ldlt().solve(-JTr);
        Eigen::Matrix4d deltaT = XYZEulertoHomogeneousMatrix(dx);
        TransformPoints(deltaT, source);
        T_icp = deltaT * T_icp;
        if(dx.norm() < epsilon) { break; }
    }
    return T_icp *initial_guess;                                                        
}

}
