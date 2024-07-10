#ifndef _FILTER_H
#define _FILTER_H

#include "human_tracking/filter_state.hpp"

namespace vk {

class Filter {
    public:
        virtual FilterState::Ptr initializeTrackState(Observation::ConstPtr observation, const VelocityVector& initialVelocity = VelocityVector::Zero()) = 0;
        virtual void predictTrackState(FilterState::Ptr state, double T) = 0;
        virtual void updateMatchedTrack(FilterState::Ptr state, Observation::ConstPtr observation) = 0;

        typedef boost::shared_ptr<Filter> Ptr;
        typedef boost::shared_ptr<const Filter> ConstPtr;
};

}

#endif /*_FILTER_H */