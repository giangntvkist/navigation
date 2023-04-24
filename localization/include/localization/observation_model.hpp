#pragma once
#include "localization/mylib.hpp"
class Laser {
    protected:
        pf_vector_t laser_pose_;
        int max_beams_;
        map_t *map_;

        bool inverse_laser_;
        double sigma_hit_;
        double z_hit_, z_rand_;
    public:
        Laser(int max_beams, map_t *map);
        void setLaserPose(pf_vector_t& laser_pose);
        virtual ~Laser();
};
class LaserData : public Laser {
    public:
        Laser *laser;
        int range_count;
        double range_min;
        double range_max;
        double angle_increment;
        double angle_min;
        double angle_max;
        double (*ranges)[2];
        LaserData() {
            ranges = nullptr;
        }
        ~LaserData() {
            delete[] ranges;
        }
};
class LikelihoodFieldModel : public Laser {
    public:
        LikelihoodFieldModel(int max_beams, map_t *map, double max_occ_dist, 
            bool inverse_laser, double sigma_hit, double z_hit, double z_rand) : Laser(int max_beams, map_t *map);
};