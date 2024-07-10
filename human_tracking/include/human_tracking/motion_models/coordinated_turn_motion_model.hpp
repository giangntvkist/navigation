#ifndef _COORDINATED_TURN_MOTION_MODEL_H_
#define _COORDINATED_TURN_MOTION_MODEL_H_

#include "human_tracking/motion_models/motion_model.hpp"
#include <ros/ros.h>

namespace vk {

class CoordinatedTurnMotionModel : public MotionModel {
    private:
        const static int DIM = 5;   // CoordinatedTurnMotionModel unknown turn rate

    public:
        CoordinatedTurnMotionModel() {
            m_A = MotionModelMatrix::Zero(DIM, DIM);
            m_Q = MotionModelMatrix::Zero(DIM, DIM);
        }

        virtual const MotionModelMatrix& A(const StateVector& x, const double T) {
            m_A = MotionModelMatrix::Identity(DIM, DIM);
            double omegak = x(STATE_OMEGA_IDX);
            double xk = x(STATE_X_IDX);
            double yk = x(STATE_Y_IDX);
            double vxk = x(STATE_VX_IDX);
            double vyk = x(STATE_VY_IDX);

            if (fabs(omegak) > 1e-6) {
                double sinwT = sin(omegak * T);
                double coswT = cos(omegak * T);

                m_A(STATE_X_IDX, STATE_VX_IDX) = sinwT / omegak;
                m_A(STATE_X_IDX, STATE_VY_IDX) = -(1 - coswT) / omegak;

                m_A(STATE_VX_IDX, STATE_VX_IDX) = coswT;
                m_A(STATE_VX_IDX, STATE_VY_IDX) = -sinwT;

                m_A(STATE_Y_IDX, STATE_VX_IDX) = (1- coswT) / omegak;
                m_A(STATE_Y_IDX, STATE_VY_IDX) = sinwT / omegak;

                m_A(STATE_VY_IDX, STATE_VX_IDX) = sinwT;
                m_A(STATE_VY_IDX, STATE_VY_IDX) = coswT;
                
                m_A(STATE_X_IDX, STATE_OMEGA_IDX) = (coswT*T*vxk)/omegak - (sinwT*vxk)/pow(omegak, 2) - (sinwT*T*vyk)/omegak - ((-1+coswT)*vyk)/pow(omegak, 2);
                m_A(STATE_Y_IDX, STATE_OMEGA_IDX) = (sinwT*T*vxk)/omegak - ((1-coswT)*vxk)/pow(omegak, 2) + (coswT*T*vyk)/omegak - (sinwT*vyk)/pow(omegak, 2);
                m_A(STATE_VX_IDX, STATE_OMEGA_IDX) = (-sinwT * T * vxk) - (coswT * T * vyk) ;
                m_A(STATE_VY_IDX, STATE_OMEGA_IDX) = (coswT * T * vxk) - (sinwT * T * vyk) ;
            }else {
                m_A(STATE_X_IDX, STATE_VX_IDX) = T;
                m_A(STATE_X_IDX, STATE_OMEGA_IDX) -0.5 * pow(T, 2) * vyk;
                m_A(STATE_VX_IDX, STATE_OMEGA_IDX) = -T * vyk;

                m_A(STATE_Y_IDX, STATE_VY_IDX) = T;
                m_A(STATE_Y_IDX, STATE_OMEGA_IDX) = -0.5 * pow(T, 2) * vxk;
                m_A(STATE_VY_IDX, STATE_OMEGA_IDX) = T * vxk;
            }
            return m_A;
        }

        // Process noise according to "Modern Tracking Systems" page 210
        virtual const MotionModelMatrix& getProcessNoiseQ(const double T, const double processNoise) {
            // get variance for turn rate omega in rad/s (default: 10 deg/s)
            const double sigmaOmega = 0.17453292519;
            m_Q = MotionModelMatrix::Zero(DIM, DIM);
            m_Q(0, 0) = pow(T, 3) * processNoise / 3.0;
            m_Q(1, 1) = m_Q(0, 0);
            m_Q(0, 2) = 0.5 * pow(T, 2) * processNoise;
            m_Q(1, 3) = m_Q(0, 2);
            m_Q(2, 0) = m_Q(0, 2);
            m_Q(3, 1) = m_Q(0, 2);
            m_Q(2, 2) = T * processNoise;
            m_Q(3, 3) = m_Q(2, 2);
            m_Q(4, 4) = sigmaOmega * T;
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

        typedef boost::shared_ptr<CoordinatedTurnMotionModel> Ptr;
        typedef boost::shared_ptr<const CoordinatedTurnMotionModel> ConstPtr;

        virtual MotionModel::Ptr deepCopy() {
            CoordinatedTurnMotionModel* copy = new CoordinatedTurnMotionModel();
            *copy = *this;
            return MotionModel::Ptr(copy);
        }
};

}

#endif /* _COORDINATED_TURN_MOTION_MODEL_H_ */