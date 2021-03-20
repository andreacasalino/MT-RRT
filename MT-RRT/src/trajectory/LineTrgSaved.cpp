/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <trajectory/LineTrgSaved.h>

namespace mt::traj {
    TargetStorer::TargetStorer(const NodeState& target)
        : targetStored(target) {
    }

    LineTrgSaved::LineTrgSaved(const NodeState& start, const NodeState& target, const float& steerDegree)
        : TargetStorer(target)
        , Line(start, this->targetStored, steerDegree) {
    }
}
