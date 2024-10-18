#pragma once
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace Eigen {
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix3x6d = Eigen::Matrix<double, 3, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
}

namespace slam {

inline Eigen::Matrix4d XYZEulertoHomogeneousMatrix(const Eigen::Vector6d& x) {
    Eigen::Matrix4d T;
    double rotX = x[3];
    double rotY = x[4];
    double rotZ = x[5];

    T(0, 0) = cos(rotZ)*cos(rotY);
    T(0, 1) = cos(rotZ)*sin(rotY)*sin(rotX) - sin(rotZ)*cos(rotX);
    T(0, 2) = cos(rotZ)*sin(rotY)*cos(rotX) + sin(rotZ)*sin(rotX);
    T(0, 3) = x[0];

    T(1, 0) = sin(rotZ)*cos(rotY);
    T(1, 1) = sin(rotZ)*sin(rotY)*sin(rotX) + cos(rotZ)*cos(rotX);
    T(1, 2) = sin(rotZ)*sin(rotY)*cos(rotX) - cos(rotZ)*sin(rotX);
    T(1, 3) = x[1];

    T(2, 0) = -sin(rotY);
    T(2, 1) = cos(rotY)*sin(rotX);
    T(2, 2) = cos(rotY)*cos(rotX);
    T(2, 3) = x[2];

    T(3, 0) = 0.0;
    T(3, 1) = 0.0;
    T(3, 2) = 0.0;
    T(3, 3) = 1.0;
    return T;
}

/* Convert 3D vector into the correspondening matrix of a lie algebra element */
inline Eigen::Matrix3d SO3intolie(const Eigen::Vector3d& p) {
    Eigen::Matrix3d m;
    m << 0.0, -p[2], p[1],
         p[2], 0.0, -p[0],
         -p[1], p[0], 0.0;
    return m;
}

}
