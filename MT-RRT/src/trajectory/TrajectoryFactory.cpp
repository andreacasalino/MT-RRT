/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <trajectory/TrajectoryFactory.h>

namespace mt::traj {
    float TrajectoryFactory::cost2Go(const NodeState& start, const NodeState& ending_node, const bool& ignoreConstraints) const {
        float distance = this->cost2GoIgnoringConstraints(start, ending_node);
        if ((ignoreConstraints) || 
            (Cost::COST_MAX == distance)) {
            return distance;
        }

        auto line = this->getTrajectory(start, ending_node);
        AdvanceInfo advInfo = AdvanceInfo::advanced;
        while (AdvanceInfo::advanced == advInfo) {
            advInfo = line->advance();
            if (AdvanceInfo::blocked == advInfo) {
                return Cost::COST_MAX;
            }
        }
        return distance;
    }

}