/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Interpolator.h>
#include <list>

namespace mt::sample {
    std::vector<NodeState> interpolate(const std::vector<NodeState>& solution, const traj::TrajectoryFactory& manager) {
        if(solution.empty()) return {};

        std::list<NodeState> states;
        states.emplace_back(solution.front());
        for(std::size_t p=1; p<solution.size(); ++p) {
            auto traj = manager.getTrajectory(states.back(), solution[p]);
            if(nullptr != traj) {
                traj::AdvanceInfo info = traj::AdvanceInfo::advanced;
                while (traj::AdvanceInfo::targetReached != info) {
                    info =  traj->advance();
                    if(traj::AdvanceInfo::blocked == info) break;
                    states.emplace_back(traj->getCursor());
                }
            }
        }

        std::vector<NodeState> result;
        result.reserve(states.size());
        for(auto it = states.begin(); it!=states.end(); ++it) {
            result.emplace_back(std::move(*it));
        }
        return result;
    }
}