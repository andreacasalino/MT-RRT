/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <node/Trajectory.h>

namespace mt::node {
    Trajectory::Trajectory(const Node& start, const Node& end)
        : start(start)
        , end(end) {
    }   
}