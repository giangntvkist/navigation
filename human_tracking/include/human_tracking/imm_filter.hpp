#ifndef _IMM_FILTER_H
#define _IMM_FILTER_H

#include "human_tracking/filter.hpp"
#include "human_tracking/ekf.hpp"
#include "human_tracking/imm_state.hpp"

namespace vk {

class IMMFilter : public Filter {
    private:
        std::vector<EKF::Ptr> m_kalmanFilters;
        int m_numberModels;
        int m_horizonal;

        double setprobability;
        IMMMatrix m_markovTransitionProbabilities;
        bool m_immVisualization;

        void computeMixingProbabilities(IMMState::Ptr state);
        void computeMixedMeanAndCovariance(IMMState::Ptr state);

        void mixPredictions(IMMState::Ptr state);
        void updateStateEstimate(IMMState::Ptr state);
        double normalDistribution(ObsVector& z, ObsVector& z_hat, ObsMatrix& S);
        void updateCurrentHypothesis(IMMState::Ptr state);
        void modeProbabilityUpdate(FilterState::Ptr state, Observation::ConstPtr observation);
    
    public:
        IMMFilter(ros::NodeHandle *nh);
        virtual FilterState::Ptr initializeTrackState(Observation::ConstPtr observation, const VelocityVector& initialVelocity = VelocityVector::Zero());
        virtual void predictTrackState(FilterState::Ptr state, double T);
        virtual void updateMatchedTrack(FilterState::Ptr state, Observation::ConstPtr observation);
        void statePredictionForHorizonal(FilterState::Ptr state, double deltatime);
};

}

#endif /* _IMM_FILTER_H */