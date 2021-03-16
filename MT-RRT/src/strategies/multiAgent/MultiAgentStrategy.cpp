/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../../Commons.h"
#include <strategies/MultiAgentStrategy.h>
#include "header/TreeMaster.h"
#include "header/TreeStarMaster.h"
#include <omp.h>
#include <atomic>
#include <Error.h>

namespace mt::solver {
    template<typename E>
    void solveParallel(std::vector<E>& battery, const std::size_t& iterations, const double& reallCoeff, multiag::TreeMaster* master) {
        std::atomic_bool life = true;
        std::size_t Threads = battery.size();
        std::size_t Batch_size = computeBatchSize(iterations, reallCoeff, Threads);

        for (size_t k = 0; k < iterations; k += Batch_size * Threads) {
            mt::sampling::SeedFactory::resetSeeds();
            master->dispatch();
#pragma omp parallel \
num_threads(static_cast<int>(Threads))
        {
            E* Solver_to_use = &battery[omp_get_thread_num()];
            Solver_to_use->extend(Batch_size);
            if (!Solver_to_use->getSolutions().empty() && (!Solver_to_use->isCumulating())) {
                life = false;
            }
#pragma omp barrier
            master->gather();
        }    
            if (!life) break;        
        }
    }

    std::unique_ptr<SolutionInfo> MultiAgentStrategy::solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>();

        auto make_extBattery = [this, &end, &sol]() -> std::vector<ExtSingle> {
            std::vector<ExtSingle> battery;
            multiag::TreeMaster* trPtr = dynamic_cast<multiag::TreeMaster*>(sol->trees.back().get());
            battery.reserve(this->solverData->problemsBattery.size());
            for (std::size_t k = 0; k < this->solverData->problemsBattery.size(); ++k) {
                battery.emplace_back(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient.get(), trPtr->getSlave(k), end);
            }
            return battery;
        };

        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<multiag::TreeMaster>(std::make_unique<Node>(start), this->solverData->problemsBattery));
            std::vector<ExtSingle> battery = make_extBattery();
            solveParallel(battery, this->parameters.Iterations_Max.get(), this->reallignmentCoeff.get(), dynamic_cast<multiag::TreeMaster*>(sol->trees.back().get()));
            fillSolutionInfo(*sol, this->parameters, battery);
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            throw Error("bidirectional startegy not possible for multi agent approach");
        }
        else {
            sol->trees.emplace_back(std::make_unique<multiag::TreeStarMaster>(std::make_unique<Node>(start), this->solverData->problemsBattery));
            std::vector<ExtSingle> battery = make_extBattery();
            solveParallel(battery, this->parameters.Iterations_Max.get(), this->reallignmentCoeff.get(), dynamic_cast<multiag::TreeMaster*>(sol->trees.back().get()));
            fillSolutionInfo(*sol, this->parameters, battery);
        }
        return sol;
    }
}