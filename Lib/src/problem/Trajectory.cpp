/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <problem/Trajectory.h>

namespace mt::problem {
    Trajectory::Trajectory(const Node& start, const Node& target)
        : start(start)
        , target(target) {
    }
}