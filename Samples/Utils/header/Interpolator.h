/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_INTERPOLATOR_H
#define MT_RRT_SAMPLE_INTERPOLATOR_H

#include <trajectory/Manager.h>

namespace mt::sample {
    std::vector<NodeState> interpolate(const std::vector<NodeState>& solution, const traj::Manager& manager);
}

#endif