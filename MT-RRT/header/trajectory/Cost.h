/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_COST_H
#define MT_RRT_COST_H

#include <Limited.h>

namespace mt::traj {
    class Cost : public Positive<float> {
    public:
        static const float COST_MAX;

        Cost();  // 0 is assumed as initial value
    };
}

#endif