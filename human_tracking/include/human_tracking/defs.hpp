#ifndef _DEFS_H
#define _DEFS_H

#define OBS_DIM 2           // observation consist only x, y
#define STATE_DIM 7
#define ROS_COV_DIM 6       // ROS pose covariances always have 6 dimensions (xyz + xyz rotation)
#define N_MODELS 3          // Number of models used for IMM filter

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>

#include <tf/tf.h>

#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>

#include <vector>

namespace vk {

typedef Eigen::Matrix<double, STATE_DIM, STATE_DIM> StateMatrix;        // square matrix of dimension STATE_DIM x STATE_DIM
typedef Eigen::Matrix<double, STATE_DIM, 1> StateVector;                // vector of dimention STATE_DIM

typedef Eigen::Matrix<double, OBS_DIM, OBS_DIM> ObsMatrix;              // square matrix of dimention OBS_DIM x OBS_DIM
typedef Eigen::Matrix<double, OBS_DIM, 1> ObsVector;                    // vector of dimention OBS_DIM

typedef Eigen::Matrix<double, OBS_DIM, STATE_DIM> ObsStateMatrix;       // matrix of dimension OBS_DIM x STATE_DIM
typedef Eigen::Matrix<double, STATE_DIM, OBS_DIM> StateObsMatrix;       // matrix of dimension STATE_DIM x OBS_DIM

typedef Eigen::Matrix<double, STATE_DIM - OBS_DIM, 1> VelocityVector;   // velocity vector at the end of a state vector

typedef Eigen::Matrix<double, N_MODELS, N_MODELS> IMMMatrix;
typedef Eigen::Matrix<double, N_MODELS, 1> IMMVector;

typedef std::vector<StateVector> StatePredict;

const unsigned int STATE_X_IDX = 0;
const unsigned int STATE_Y_IDX = 1;
const unsigned int STATE_VX_IDX = 2;
const unsigned int STATE_VY_IDX = 3;
const unsigned int STATE_OMEGA_IDX = 4;
const unsigned int STATE_AX_IDX = 5;
const unsigned int STATE_AY_IDX = 6;

const unsigned int CV_IDX = 0;
const unsigned int CA_IDX = 1;
const unsigned int CT_IDX = 2;

}

#endif      // _DEFS_H