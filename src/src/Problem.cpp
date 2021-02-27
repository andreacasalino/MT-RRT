/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Problem.h>
#include <Error.h>

namespace mt {
    Problem::Problem(sampling::SamplerPtr sampler, traj::ManagerPtr manager, const std::size_t& stateSpaceSize, const float& gamma, const bool& simmetry)
        : stateSpaceSize(stateSpaceSize)
        , gamma(gamma)
        , simmetry(simmetry) {
        if (this->stateSpaceSize <= 1) throw Error("invalid state space size");
        if (nullptr == sampler) throw Error("found null sampler");
        this->sampler = std::move(sampler);
        if (nullptr == manager) throw Error("found null trajectory manager");
        this->trajManager = std::move(trajManager);
        if (this->gamma < 0.f) throw Error("invalid negative gamma");
    }

    Problem::Problem(const Problem& o)
        : Problem(o.sampler->copy(), o.trajManager->copy(), o.stateSpaceSize, o.gamma, o.simmetry) {
    }

    void Problem::setSteerTrials(const std::size_t& trials) {
        if (0 == trials) {
            throw Error("0 is invalid as steer trials");
        }
        this->steerTrials = trials;
    }

    NodePtr Problem::steer(Node& start, const NodeState& trg, bool& trg_reached) {
        traj::TrajectoryPtr traj = this->trajManager->getTrajectory(start.getState() , trg);
        NodeState steered;
        trg_reached = false;
        for (std::size_t t = 0; t < this->steerTrials; ++t) {
            auto info = traj->advance();
            if (traj::Trajectory::advanceInfo::blocked == info) break;
            steered = traj->getCursor();
            if (traj::Trajectory::advanceInfo::targetReached == info) {
                trg_reached = true;
                break;
            }
        }
        if (steered.empty()) return nullptr;
        NodePtr ptr = std::make_unique<Node>(steered);
        ptr->setFather(&start, traj->getCummulatedCost());
        return ptr;
    }
}