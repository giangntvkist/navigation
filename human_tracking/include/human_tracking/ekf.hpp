#ifndef _EKF_H
#define _EKF_H

#include "human_tracking/filter.hpp"
#include "human_tracking/motion_models/motion_model.hpp"

namespace vk {

class EKF : public Filter {
    private:
        StateMatrix m_A;
        StateMatrix m_initialC;
        StateMatrix m_defaultQ;

        bool m_useProcessNoise;
        double m_processNoiseValue;

        MotionModel::Ptr m_motionModel;

    public:
        EKF(const unsigned int MODEL_IDX);
        void initializeTrackState(FilterState::Ptr state, Observation::ConstPtr observation, const VelocityVector& initialVelocity = VelocityVector::Zero());
        virtual FilterState::Ptr initializeTrackState(Observation::ConstPtr observation, const VelocityVector& initialVelocity = VelocityVector::Zero());
        virtual void predictTrackState(FilterState::Ptr state, double T);
        virtual void updateMatchedTrack(FilterState::Ptr state, Observation::ConstPtr observation);

        void statePrediction(FilterState::Ptr state, double deltatime);

        typedef boost::shared_ptr<EKF> Ptr;
        typedef boost::shared_ptr<const EKF> ConstPtr;
};

}

#endif /* _EKF_H */