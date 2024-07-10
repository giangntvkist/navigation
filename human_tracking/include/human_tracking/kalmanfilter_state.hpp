#ifndef _KALMANFILTER_STATE_H
#define _KALMANFILTER_STATE_H

#include "human_tracking/filter_state.hpp"

namespace vk {

class EKF;
class IMMFilter;
class IMMState;

class KalmanFilterState : public FilterState {
    private:
        StateVector m_xp;
        StateMatrix m_Cp;
    protected:
        StateVector m_x;
        StateMatrix m_C;

        StateVector m_x_;
        StateVector m_xp_;

    public:
        virtual const StateVector& x() { return m_x; };
        virtual const StateMatrix& C() { return m_C; };
        virtual const StateVector& xp() { return m_xp; };
        virtual const StateMatrix& Cp() { return m_Cp; };

        void setStateValue() {
            m_x_ = m_x;
        }
        void setPredictValue() {
            m_x_ = m_xp_;
        }

        typedef boost::shared_ptr<KalmanFilterState> Ptr;
        typedef boost::shared_ptr<const KalmanFilterState> ConstPtr;

        friend class EKF;
        friend class IMMFilter;
        friend class IMMState;
};

}

#endif /* _KALMANFILTER_STATE_H */