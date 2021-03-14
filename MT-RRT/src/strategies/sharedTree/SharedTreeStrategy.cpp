/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../../Commons.h"
#include <strategies/SharedTreeStrategy.h>
#include "header/TreeShared.h"
#include "header/TreeStarShared.h"
#include <omp.h>
#include <atomic>

namespace mt::solver {
    template<typename E>
    void solveParallel(std::vector<E>& battery, const std::size_t& iterations) {
        std::atomic_bool life = true;
        mt::sampling::SeedFactory::resetSeeds();
#pragma omp parallel \
num_threads(static_cast<int>(battery.size()))
        {
            std::size_t Number_threads = static_cast<std::size_t>(omp_get_num_threads());
            E* Solver_to_use = &battery[omp_get_thread_num()];
            for (size_t k = 0; k < iterations; k += Number_threads) {
                if (!life) break;
                Solver_to_use->extend(1);
                if (!Solver_to_use->getSolutions().empty() && (!Solver_to_use->isCumulating())) {
                    life = false;
                }
            }
        }
    }

    std::unique_ptr<SolutionInfo> SharedTreeStrategy::solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>();

        auto make_extBattery1 = [this, &end, &sol]() -> std::vector<ExtSingle> {
            std::vector<ExtSingle> battery;
            battery.reserve(this->solverData->problemsBattery.size());
            for (std::size_t k = 0; k < this->solverData->problemsBattery.size(); ++k) {
                battery.emplace_back(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient.get(), *sol->trees.front().get(), end);
            }
            return battery;
        };

        auto make_extBattery2 = [this, &sol]() -> std::vector<ExtBidir> {
            std::vector<ExtBidir> battery;
            battery.reserve(this->solverData->problemsBattery.size());
            for (std::size_t k = 0; k < this->solverData->problemsBattery.size(); ++k) {
                battery.emplace_back(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient.get(), *sol->trees.front().get(), *sol->trees.back().get());
            }
            return battery;
        };

        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<shared::TreeShared>(std::make_unique<Node>(start) , this->solverData->problemsBattery ));
            std::vector<ExtSingle> battery = make_extBattery1();
            solveParallel(battery, this->parameters.Iterations_Max.get());
            fillSolutionInfo(*sol, this->parameters, battery);
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<shared::TreeShared>(std::make_unique<Node>(start) , this->solverData->problemsBattery));
            sol->trees.emplace_back(std::make_unique<shared::TreeShared>(std::make_unique<Node>(end) , this->solverData->problemsBattery));
            std::vector<ExtBidir> battery = make_extBattery2();
            solveParallel(battery, this->parameters.Iterations_Max.get());
            fillSolutionInfo(*sol, this->parameters, battery);
        }
        else {
            sol->trees.emplace_back(std::make_unique<shared::TreeStarShared>(std::make_unique<Node>(start), this->solverData->problemsBattery));
            std::vector<ExtSingle> battery = make_extBattery1();
            solveParallel(battery, this->parameters.Iterations_Max.get());
            fillSolutionInfo(*sol, this->parameters, battery);
        }
        return sol;
    }
}