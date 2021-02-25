/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <problem/Problem.h>
#include <problem/Sampler.h>
#include <problem/Trajectory.h>
#include <Error.h>

namespace mt::problem {
    Problem::Problem(SamplerPtr sampler, CheckerPtr checker, const std::size_t& stateSpaceSize, const float& gamma, const bool& simmetry)
        : stateSpaceSize(stateSpaceSize)
        , gamma(gamma)
        , simmetry(simmetry) {
        if (0 == this->stateSpaceSize) throw Error("invalid zero state space size");
        if (nullptr == sampler) throw Error("found null sampler");
        this->sampler = std::move(sampler);
        if (nullptr == checker) throw Error("found null checker");
        this->checker = std::move(checker);
        if (this->gamma < 0.f) throw Error("invalid negative gamma");
    }

    Problem::Problem(const Problem& o)
        : Problem(o.sampler->copy(), o.checker->copy(), o.stateSpaceSize, o.gamma, o.simmetry) {
    }

    NodeState Problem::randomState() {
        return this->sampler->randomState();
    }

    void Problem::setSteerTrials(const std::size_t& trials) {
        if (0 == trials) {
            throw Error("0 is invalid as steer trials");
        }
        this->steerTrials = trials;
    }

    NodePtr Problem::steer(Node& start, const NodeState& trg, bool& trg_reached) {
        TrajectoryPtr traj = this->getTrajectory(start.getState(), trg);
        NodeState steered;
        trg_reached = false;
        for (std::size_t t = 0; t < this->steerTrials; ++t) {
            if (this->checker->isNotAdmitted(traj->getCursor())) {
                break;
            }
            steered = traj->getCursor();
            if (traj->isCursorAtEnd()) {
                trg_reached = true;
                break;
            }
            traj->advanceCursor();
        }
        if (steered.empty()) return nullptr;
        NodePtr ptr = std::make_unique<Node>(steered);
        ptr->setFather(&start, traj->getCummulatedCost());
        return ptr;
    }
}