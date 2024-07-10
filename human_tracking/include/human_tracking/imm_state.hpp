#ifndef _IMM_STATE_H
#define _IMM_STATE_H

#include "human_tracking/filter_state.hpp"
#include "human_tracking/imm_hypothesis.hpp"

namespace vk {

class IMMFilter;

typedef unsigned int IMMHypothesisIndex;

class IMMState : public FilterState {
    private:
        StateVector m_xp;
        StateMatrix m_Cp;
        StateVector m_x;
        StateMatrix m_C;

        StateVector m_x_;
        StatePredict m_statePrediction;

        IMMHypotheses m_hypotheses;
        IMMHypothesis::Ptr m_currentHypothesis;
        IMMHypothesisIndex m_currentHypothesisIdx;

        IMMMatrix m_mixingProbabilities;
    public:
        virtual const StateVector& x() { return m_x; };
        virtual const StateMatrix& C() { return m_C; };
        virtual const StateVector& xp() { return m_xp; };
        virtual const StateMatrix& Cp() { return m_Cp; };
        StatePredict& getStatePrediction() { return m_statePrediction; };

        typedef boost::shared_ptr<IMMState> Ptr;
        typedef boost::shared_ptr<const IMMState> ConstPtr;

        friend class IMMFilter;
};

}

#endif /* _IMM_STATE_H */