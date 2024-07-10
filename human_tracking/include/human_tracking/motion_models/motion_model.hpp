#ifndef _MOTION_MODEL_H_
#define _MOTION_MODEL_H_

#include "human_tracking/defs.hpp"

namespace vk {

class MotionModel {
    public:
        typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MotionModelMatrix;
        typedef Eigen::Matrix<double, Eigen::Dynamic, 1> MotionModelVector;

        virtual const MotionModelMatrix& A(const StateVector& x, const double T) = 0;
        virtual const MotionModelMatrix& getProcessNoiseQ(const double T, const double processNoise) = 0;

        virtual const MotionModelMatrix convertToMotionModel(const StateMatrix& matrix) = 0;
        virtual const MotionModelVector convertToMotionModel(const StateVector& vector) = 0;

        virtual const StateMatrix convertToState(const MotionModelMatrix& matrix) = 0;
        virtual const StateVector convertToState(const MotionModelVector& vector) = 0;

        typedef boost::shared_ptr<MotionModel> Ptr;
        typedef boost::shared_ptr<const MotionModel> ConstPtr;

        virtual Ptr deepCopy() = 0;

    protected:
        MotionModelMatrix m_A;
        MotionModelMatrix m_Q;
};

}

#endif /* _MOTION_MODEL_H_ */