/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Problem.h>
#include <Error.h>

namespace mt {
    Problem::Problem(sampling::SamplerPtr sampler, traj::TrajectoryManagerPtr manager, const std::size_t& stateSpaceSize, const float& gamma, const bool& simmetry)
        : stateSpaceSize(1, stateSpaceSize)
        , gamma(gamma)
        , simmetry(simmetry) {
        if (nullptr == sampler) throw Error("found null sampler");
        this->sampler = std::move(sampler);
        if (nullptr == manager) throw Error("found null trajectory manager");
        this->trajManager = std::move(manager);
    }

    Problem::Problem(const Problem& o)
        : Problem(o.sampler->copy(), o.trajManager->copy(), o.stateSpaceSize.get(), o.gamma.get(), o.simmetry) {
        this->steerTrials.set(o.steerTrials.get());
    }

    NodePtr Problem::steer(Node& start, const NodeState& trg, bool& trg_reached) {
        traj::TrajectoryPtr traj = this->trajManager->getTrajectory(start.getState() , trg);
        NodeState steered;
        trg_reached = false;
        traj::Trajectory::AdvanceInfo info;
        for (std::size_t t = 0; t < this->steerTrials.get(); ++t) {
            info = traj->advance();
            if (traj::Trajectory::AdvanceInfo::blocked == info) break;
            steered = traj->getCursor();
            if (traj::Trajectory::AdvanceInfo::targetReached == info) {
                trg_reached = true;
                break;
            }
        }
        if (steered.empty()) return nullptr;
        NodePtr ptr = std::make_unique<Node>(steered);
        ptr->setFather(&start, traj->getCumulatedCost());
        return ptr;
    }
}