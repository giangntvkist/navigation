#include "human_tracking/ekf.hpp"
#include "human_tracking/kalmanfilter_state.hpp"
#include "human_tracking/motion_models/constant_velocity_motion_model.hpp"
#include "human_tracking/motion_models/acceleration_motion_model.hpp"
#include "human_tracking/motion_models/coordinated_turn_motion_model.hpp"

namespace vk {

EKF::EKF(const unsigned int MODEL_IDX) {
    m_initialC = StateMatrix::Zero();
    m_defaultQ = StateMatrix::Zero();
    m_A        = StateMatrix::Zero();
    
    m_initialC(STATE_X_IDX, STATE_X_IDX) = 0.7;
    m_initialC(STATE_Y_IDX, STATE_Y_IDX) = 0.7;
    if(MODEL_IDX == CV_IDX) {
        m_motionModel.reset(new ConstantMotionModel);
        m_initialC(STATE_VX_IDX, STATE_VX_IDX) = 2.0;
        m_initialC(STATE_VY_IDX, STATE_VY_IDX) = 2.0;
    }else if(MODEL_IDX == CA_IDX) {
        m_motionModel.reset(new AccelerationMotionModel);
        m_initialC(STATE_VX_IDX, STATE_VX_IDX) = 2.0;
        m_initialC(STATE_VY_IDX, STATE_VY_IDX) = 2.0;
        m_initialC(STATE_AX_IDX, STATE_AX_IDX) = 0.2;
        m_initialC(STATE_AY_IDX, STATE_AY_IDX) = 0.2;
    }else if(MODEL_IDX == CT_IDX) {
        m_motionModel.reset(new CoordinatedTurnMotionModel);
        m_initialC(STATE_VX_IDX, STATE_VX_IDX) = 2.0;
        m_initialC(STATE_VY_IDX, STATE_VY_IDX) = 2.0;
        m_initialC(STATE_OMEGA_IDX, STATE_OMEGA_IDX) = 0.8;
    }else {
        ROS_ERROR("MOTION MODEL INDEX is unknown!");
    }

    m_useProcessNoise = true;
    if(m_useProcessNoise) {
       m_processNoiseValue = 0.5; 
    }else {
        m_defaultQ(STATE_X_IDX, STATE_X_IDX) = 0.0081;
        m_defaultQ(STATE_Y_IDX, STATE_Y_IDX) = 0.0081;
        m_defaultQ(STATE_VX_IDX, STATE_VX_IDX) = 0.0064;
        m_defaultQ(STATE_VY_IDX, STATE_VY_IDX) = 0.0064;
    }
}

void EKF::initializeTrackState(FilterState::Ptr state, Observation::ConstPtr observation, const VelocityVector& initialVelocity) {
    KalmanFilterState::Ptr kfs = boost::dynamic_pointer_cast<KalmanFilterState>(state);
    kfs->m_x = StateVector::Zero();
    kfs->m_x.head(OBS_DIM) = observation->z;
    kfs->m_x.tail(STATE_DIM - OBS_DIM) = initialVelocity;

    kfs->m_C = m_initialC;
    kfs->m_xp = kfs->m_x;
    kfs->m_Cp = kfs->m_C;
}

FilterState::Ptr EKF::initializeTrackState(Observation::ConstPtr observation, const VelocityVector& initialVelocity) {
    KalmanFilterState::Ptr kfs(new KalmanFilterState);
    initializeTrackState(kfs, observation, initialVelocity);
    return kfs;
}

void EKF::predictTrackState(FilterState::Ptr state, double T) {
    KalmanFilterState& kfs = dynamic_cast<KalmanFilterState&>(*state);
    MotionModel::MotionModelMatrix Q;
    if(m_useProcessNoise) {
        Q = m_motionModel->getProcessNoiseQ(T, m_processNoiseValue);
    }else {
        Q = m_defaultQ;
    }
    MotionModel::MotionModelVector x, xp;
    MotionModel::MotionModelMatrix A, Cp;

    A = m_motionModel->A(kfs.m_x, T);
    xp = A * m_motionModel->convertToMotionModel(kfs.m_x);
    Cp = A * m_motionModel->convertToMotionModel(kfs.m_C) * A.transpose() + Q;

    kfs.m_xp = m_motionModel->convertToState(xp);
    kfs.m_Cp = m_motionModel->convertToState(Cp);
}

void EKF::updateMatchedTrack(FilterState::Ptr state, Observation::ConstPtr observation) {
    KalmanFilterState& kfs = dynamic_cast<KalmanFilterState&>(*state);
    kfs.m_H = ObsStateMatrix::Zero();
    kfs.m_H(STATE_X_IDX, STATE_X_IDX) = 1;
    kfs.m_H(STATE_Y_IDX, STATE_Y_IDX) = 1;
    ObsMatrix S = kfs.m_H * kfs.m_Cp * kfs.m_H.transpose() + observation->R;
    StateObsMatrix K = kfs.m_Cp * kfs.m_H.transpose() * S.inverse();
    kfs.m_x = kfs.m_xp + K * (observation->z - kfs.m_H * kfs.m_xp);
    kfs.m_C = kfs.m_Cp - K * kfs.m_H * kfs.m_Cp;
}

void EKF::statePrediction(FilterState::Ptr state, double T) {
    KalmanFilterState& kfs = dynamic_cast<KalmanFilterState&>(*state);
    MotionModel::MotionModelVector xp;
    MotionModel::MotionModelMatrix A;

    A = m_motionModel->A(kfs.m_x_, T);
    xp = A * m_motionModel->convertToMotionModel(kfs.m_x_);
    kfs.m_xp_ = m_motionModel->convertToState(xp);
}

}
