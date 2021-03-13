/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../../Commons.h"
#include <strategies/LinkedTreesStrategy.h>
#include "header/TreeLinked.h"
#include "header/TreeStarLinked.h"
#include "header/Gatherer.h"
#include <omp.h>
#include <atomic>

namespace mt::solver {
    template<typename E>
    void solveParallel(std::vector<E>& battery, const std::size_t& iterations, const double& reallCoeff, const linked::Gatherer& gatherer) {
        std::atomic_bool life = true;
        std::size_t Threads = battery.size();
        std::size_t Batch_size = computeBatchSize(iterations, reallCoeff, Threads);

#pragma omp parallel \
num_threads(static_cast<int>(Threads))
        {
            E* Solver_to_use = &battery[omp_get_thread_num()];
            for (size_t k = 0; k < iterations; k += Batch_size * Threads) {
                if (!life) break;
                Solver_to_use->extend(Batch_size);
                if (!Solver_to_use->getSolutions().empty() && (!Solver_to_use->isCumulating())) {
                    life = false;
                }
#pragma omp barrier
                gatherer.gather();
            }
#pragma omp barrier
            gatherer.gather();
        }
    }

    std::unique_ptr<SolutionInfo> LinkedTreesStrategy::solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>();

        auto make_extBattery1 = [this, &end, &sol]() -> std::vector<ExtSingle> {
            std::vector<ExtSingle> battery;
            battery.reserve(this->solverData->problemsBattery.size());
            for (std::size_t k = 0; k < this->solverData->problemsBattery.size(); ++k) {
                battery.emplace_back(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient.get(), *sol->trees[k], end);
            }
            return battery;
        };

        auto make_extBattery2 = [this, &sol](const std::vector<TreePtr>& otherTrees) -> std::vector<ExtBidir> {
            std::vector<ExtBidir> battery;
            battery.reserve(this->solverData->problemsBattery.size());
            for (std::size_t k = 0; k < this->solverData->problemsBattery.size(); ++k) {
                battery.emplace_back(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient.get(), *sol->trees[k], *otherTrees[k]);
            }
            return battery;
        };


        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees = linked::TreeLinked::make_trees(std::make_unique<Node>(start), this->solverData->problemsBattery);
            std::vector<ExtSingle> battery = make_extBattery1();
            solveParallel(battery, this->parameters.Iterations_Max.get(), this->reallignmentCoeff.get(), linked::Gatherer(sol->trees));
            fillSolutionInfo(*sol, this->parameters, battery);
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            sol->trees = linked::TreeLinked::make_trees(std::make_unique<Node>(start), this->solverData->problemsBattery);
            auto temp = linked::TreeLinked::make_trees(std::make_unique<Node>(end), this->solverData->problemsBattery);
            linked::GathererBid gatherer(sol->trees, temp);
            std::vector<ExtBidir> battery = make_extBattery2(temp);
            {
                sol->trees.reserve(this->solverData->problemsBattery.size() * 2);
                for (auto it = temp.begin(); it != temp.end(); ++it) {
                    sol->trees.emplace_back(std::move(*it));
                }
            }
            solveParallel(battery, this->parameters.Iterations_Max.get(), this->reallignmentCoeff.get(), gatherer);
            fillSolutionInfo(*sol, this->parameters, battery);
        }
        else {
            sol->trees = linked::TreeStarLinked::make_trees(std::make_unique<Node>(start), this->solverData->problemsBattery);
            std::vector<ExtSingle> battery = make_extBattery1();
            solveParallel(battery, this->parameters.Iterations_Max.get(), this->reallignmentCoeff.get(), linked::Gatherer(sol->trees));
            fillSolutionInfo(*sol, this->parameters, battery);
        }
        return sol;
    }
}