/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Line2.h"

namespace mt::traj {
    TargetStore::TargetStore(const NodeState& target)
        : targetState(target) {
    }

    Line2::Line2(const NodeState& start, const NodeState& target, const float& steerDegree)
        : TargetStore(target)
        , Line(this->targetState, target, steerDegree) {
    }
}
