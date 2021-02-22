/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <problem/Trajectory.h>
#include <limits>

namespace mt::problem {
    const float Trajectory::COST_MAX = std::numeric_limits<float>::max();

    Trajectory::Trajectory(const Node& start, const Node& target)
        : start(start)
        , target(target) {
    }
}