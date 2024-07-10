#ifndef _OBSERVATION_H
#define _OBSERVATION_H

#include "human_tracking/defs.hpp"
#include <vector>

namespace vk {

typedef unsigned int observation_id;

struct Observation {
    observation_id id;  
    ObsVector z;
    ObsMatrix R;
    double confidence;

    typedef boost::shared_ptr<Observation> Ptr;
    typedef boost::shared_ptr<const Observation> ConstPtr;
};

typedef std::vector<Observation::Ptr> Observations;

}

#endif /* _OBSERVATION_H */