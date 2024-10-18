#ifndef CONSTANT_VELOCITY_MOTION_MODEL_H_
#define CONSTANT_VELOCITY_MOTION_MODEL_H_

#include "human_tracking/motion_models/motion_model.hpp"
#include <ros/ros.h>

namespace vk {

class ConstantMotionModel : public MotionModel {
    private:
        const static int DIM = 4;
    public:
    ConstantMotionModel() {
        m_A = MotionModelMatrix::Zero(DIM, DIM);
        m_Q = MotionModelMatrix::Zero(DIM, DIM);
    }

    virtual const MotionModelMatrix& A (const StateVector& x, const double T) {
        m_A = MotionModelMatrix::Identity(DIM, DIM);
        m_A(STATE_X_IDX, STATE_VX_IDX) = T;
        m_A(STATE_Y_IDX, STATE_VY_IDX) = T;
        return m_A;
    }

    virtual const MotionModelMatrix& getProcessNoiseQ(const double T, const double processNoise) {
        m_Q = MotionModelMatrix::Zero(DIM, DIM);
        m_Q(STATE_X_IDX, STATE_X_IDX) = pow(T, 3) * processNoise / 3.0;
        m_Q(STATE_X_IDX, STATE_VX_IDX) = 0.5 * pow(T, 2) * processNoise;
        m_Q(STATE_Y_IDX, STATE_Y_IDX) = m_Q(STATE_X_IDX, STATE_X_IDX);
        m_Q(STATE_Y_IDX, STATE_VY_IDX) = m_Q(STATE_X_IDX, STATE_VX_IDX);
        m_Q(STATE_VX_IDX, STATE_X_IDX) = m_Q(STATE_X_IDX, STATE_VX_IDX);
        m_Q(STATE_VX_IDX, STATE_VX_IDX) = T * processNoise;
        m_Q(STATE_VY_IDX, STATE_Y_IDX) = m_Q(STATE_X_IDX, STATE_VX_IDX);
        m_Q(STATE_VY_IDX, STATE_VY_IDX) = m_Q(STATE_VX_IDX, STATE_VX_IDX);
        return m_Q;
    }

    virtual const MotionModelMatrix convertToMotionModel(const StateMatrix& matrix) {
        MotionModelMatrix model = matrix.block<DIM, DIM>(0, 0);
        return model;
    }

    virtual const MotionModelVector convertToMotionModel(const StateVector& vector) {
        MotionModelVector model = vector.head(DIM);
        return model;
    }

    virtual const StateMatrix convertToState(const MotionModelMatrix& matrix) {
        StateMatrix state = StateMatrix::Identity();
        state.block<DIM, DIM>(0, 0) = matrix.block<DIM, DIM>(0, 0);
        return state;
    }
    
    virtual const StateVector convertToState(const MotionModelVector& vector) {
        StateVector state = StateVector::Zero();
        state.head(DIM) = vector.head(DIM);
        return state;
    }

    typedef boost::shared_ptr<ConstantMotionModel> Ptr;
    typedef boost::shared_ptr<const ConstantMotionModel> ConstPtr;
    
    virtual MotionModel::Ptr deepCopy() {
        ConstantMotionModel* copy = new ConstantMotionModel();
        *copy = *this;
        return MotionModel::Ptr(copy);
    }
};


}

#endif /* CONSTANT_VELOCITY_MOTION_MODEL_H_ */