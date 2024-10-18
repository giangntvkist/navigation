#ifndef _ACCELERATION_MOTION_MODEL_H_
#define _ACCELERATION_MOTION_MODEL_H_

#include "human_tracking/motion_models/motion_model.hpp"
#include <ros/ros.h>

namespace vk {

class AccelerationMotionModel : public MotionModel {
    private:
        const static int DIM = 6;
        const static unsigned int IDX_X = 0;
        const static unsigned int IDX_Y = 1;
        const static unsigned int IDX_VX = 2;
        const static unsigned int IDX_VY = 3;
        const static unsigned int IDX_AX = 4;
        const static unsigned int IDX_AY = 5;

        void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove) {
            unsigned int numRows = matrix.rows()-1;
            unsigned int numCols = matrix.cols();
            if( rowToRemove < matrix.rows()-1 ) {
                matrix.block(rowToRemove, 0, numRows-rowToRemove, numCols) = matrix.block(rowToRemove+1, 0, numRows-rowToRemove, numCols);
            }
            matrix.conservativeResize(numRows, numCols);
        }

        void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove) {
            unsigned int numRows = matrix.rows();
            unsigned int numCols = matrix.cols()-1;
            if( colToRemove < matrix.cols()-1 ) {
                matrix.block(0,colToRemove, numRows, numCols-colToRemove) = matrix.block(0,colToRemove+1, numRows, numCols-colToRemove);
            }
            matrix.conservativeResize(numRows, numCols);
        }
    public:
        AccelerationMotionModel() {
            m_A = MotionModelMatrix::Zero(DIM, DIM);
            m_Q = MotionModelMatrix::Zero(DIM, DIM);
        }

        virtual const MotionModelMatrix& A (const StateVector& x, const double T) {
            m_A = MotionModelMatrix::Identity(DIM, DIM);
            m_A(IDX_X, IDX_VX) = T;
            m_A(IDX_Y, IDX_VY) = T;
            m_A(IDX_X, IDX_AX) = 0.5 * pow(T, 2);
            m_A(IDX_Y, IDX_AY) = 0.5 * pow(T, 2);
            m_A(IDX_VX, IDX_AX) = T;
            m_A(IDX_VY, IDX_AY) = T;
            return m_A;
        }

        virtual const MotionModelMatrix& getProcessNoiseQ(const double T, const double processNoise) {
            m_Q = MotionModelMatrix::Zero(DIM, DIM);
            m_Q(IDX_X, IDX_X) = pow(T, 5) * processNoise / 20.0;
            m_Q(IDX_X, IDX_VX) = pow(T, 4) * processNoise / 8.0;
            m_Q(IDX_X, IDX_AX) = pow(T, 3) * processNoise / 6.0;
            
            m_Q(IDX_Y, IDX_Y) = m_Q(IDX_X, IDX_X);
            m_Q(IDX_Y, IDX_VY) = m_Q(IDX_X, IDX_VX);
            m_Q(IDX_Y, IDX_AY) = m_Q(IDX_X, IDX_AX);

            m_Q(IDX_VX, IDX_X) = m_Q(IDX_X, IDX_VX);
            m_Q(IDX_VX, IDX_VX) = pow(T, 3) * processNoise / 3.0;
            m_Q(IDX_VX, IDX_AX) = pow(T, 2) * processNoise / 2.0;

            m_Q(IDX_VY, IDX_Y) = m_Q(IDX_X, IDX_VX);
            m_Q(IDX_VY, IDX_VY) = m_Q(IDX_VX, IDX_VX);
            m_Q(IDX_VY, IDX_AY) = m_Q(IDX_VX, IDX_AX);
            
            m_Q(IDX_AX, IDX_X) = m_Q(IDX_X, IDX_AX);
            m_Q(IDX_AX, IDX_VX) = m_Q(IDX_VX, IDX_AX);
            m_Q(IDX_AX, IDX_AX) = T * processNoise;

            m_Q(IDX_AY, IDX_Y) = m_Q(IDX_X, IDX_AX);
            m_Q(IDX_AY, IDX_VY) = m_Q(IDX_VX, IDX_AX);
            m_Q(IDX_AY, IDX_AY) = m_Q(IDX_AX, IDX_AX);

            return m_Q;
        }

        virtual const MotionModelMatrix convertToMotionModel(const StateMatrix& matrix) {
            MotionModelMatrix model = matrix;
            removeColumn(model, STATE_OMEGA_IDX);
            removeRow(model, STATE_OMEGA_IDX);
            return model;
        }

        virtual const MotionModelVector convertToMotionModel(const StateVector& vector) {
            MotionModelVector model = MotionModelVector::Zero(DIM);
            model.head(4) = vector.head(4);
            model.tail(2) = vector.tail(2);
            return model;
        }

        virtual const StateMatrix convertToState(const MotionModelMatrix& matrix) {
            StateMatrix state = StateMatrix::Identity();
            state.block<4,4>(0, 0) = matrix.block<4,4>(0, 0);
            state.block<2,4>(STATE_AX_IDX, STATE_X_IDX) = matrix.block<2,4>(IDX_AX, IDX_X);
            state.block<4,2>(STATE_X_IDX,STATE_AX_IDX) = matrix.block<4,2>(IDX_X,IDX_AX);
            state.block<2,2>(STATE_AX_IDX,STATE_AX_IDX) = matrix.block<2,2>(IDX_AX,IDX_AX);
            return state;
        }
    
        virtual const StateVector convertToState(const MotionModelVector& vector) {
            StateVector state = StateVector::Zero();
            state.head(4) = vector.head(4);
            state.tail(2) = vector.tail(2);
            return state;
        }

        typedef boost::shared_ptr<AccelerationMotionModel> Ptr;
        typedef boost::shared_ptr<const AccelerationMotionModel> ConstPtr;
    
    virtual MotionModel::Ptr deepCopy() {
        AccelerationMotionModel* copy = new AccelerationMotionModel();
        *copy = *this;
        return MotionModel::Ptr(copy);
    }

};

}


#endif /* _ACCELERATION_MOTION_MODEL_H_ */