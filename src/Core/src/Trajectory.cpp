/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Trajectory.h>
#include <limits>

namespace mt {
    const float Trajectory::COST_MAX = std::numeric_limits<float>::max();

    Trajectory::Trajectory(const NodeState& start, const NodeState& target)
        : start(start)
        , target(target) {
    }
}