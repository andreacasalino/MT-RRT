/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <trajectory/Trajectory.h>
#include <limits>

namespace mt::traj {
    Trajectory::Trajectory(const NodeState& start, const NodeState& target)
        : target(target)
        , cursor(start) {
    }
}