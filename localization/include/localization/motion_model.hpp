#pragma once
#include "localization/mylib.hpp"
class MotionModel {
    public:
        virtual void initialize(double anpha1, double anpha2, double anpha3, double anpha4, double anpha5) = 0;
        virtual void odometryUpdate(pf_set_sample_t *pf, pf_vector_t& pose, pf_vector_t& odom_t, pf_vector_t& odom_t_1) = 0;
        virtual ~MotionModel() = default;
};

class OmniModel : public MotionModel {
    private:
        double anpha1_, anpha2_, anpha3_, anpha4_, anpha5_;
    public:
        void initialize(double anpha1, double anpha2, double anpha3, double anpha4, double anpha5);
        void odometryUpdate(pf_set_sample_t *pf, pf_vector_t& pose, pf_vector_t& odom_t, pf_vector_t& odom_t_1);
};

class DiffModel : public MotionModel {
    private:
        double anpha1_, anpha2_, anpha3_, anpha4_, anpha5_;
    public:
        void initialize(double anpha1, double anpha2, double anpha3, double anpha4, double anpha5);
        void odometryUpdate(pf_set_sample_t *pf, pf_vector_t& pose, pf_vector_t& odom_t, pf_vector_t& odom_t_1);
};


