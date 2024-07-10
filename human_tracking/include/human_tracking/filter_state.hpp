#ifndef _FILTER_STATE_H
#define _FILTER_STATE_H

#include "human_tracking/defs.hpp"
#include "human_tracking/observation.hpp"

namespace vk {

class FilterState {
    protected:
        ObsVector m_zp;
        ObsStateMatrix m_H;
    public:
        virtual const StateVector& x() = 0;
        virtual const StateMatrix& C() = 0;
        virtual const StateVector& xp() = 0;
        virtual const StateMatrix& Cp() = 0;

        typedef boost::shared_ptr<FilterState> Ptr;
        typedef boost::shared_ptr<const FilterState> ConstPtr;
};

}

#endif /* _FILTER_STATE_H */