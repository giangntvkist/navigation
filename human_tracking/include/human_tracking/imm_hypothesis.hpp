#ifndef _IMM_HYPOTHESIS_H
#define _IMM_HYPOTHESIS_H

#include <vector>
#include "human_tracking/kalmanfilter_state.hpp"

namespace  vk {

class IMMFilter;

class IMMHypothesis : public KalmanFilterState {
    private:
        StateVector m_xMixed;
        StateMatrix m_CMixed;
        double m_probability;
    
    public:
        const StateVector& xMixed() const { return m_xMixed; }
        const StateMatrix& CMixed() const { return m_CMixed; }

        double getProbability() { return m_probability; }

        void setMixedValuesForState() {
            m_x = m_xMixed;
            m_C = m_CMixed;
        }

        typedef boost::shared_ptr<IMMHypothesis> Ptr;
        typedef boost::shared_ptr<const IMMHypothesis> ConstPtr;

        friend class IMMFilter;
};

typedef std::vector<IMMHypothesis::Ptr> IMMHypotheses;

}

#endif /* _IMM_HYPOTHESIS_H */