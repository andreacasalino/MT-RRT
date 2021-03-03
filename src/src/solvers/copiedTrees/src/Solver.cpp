/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Solver.h>
#include <ExtenderSingle.h>
#include <ExtenderBidir.h>
#include "../TreeConcreteLinked.h"
#include "../TreeStarLinked.h"
#include <atomic>
#include <math.h>
#include <omp.h>
#include "../Gatherer.h"
#include "../Commons.h"

namespace mt {
    template<typename E>
    void solveParallel(std::vector<E>& battery, const std::size_t& iterations, const double& reallCoeff, const copied::Gatherer& gatherer) {
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
#pragma omp barrier
            }
            gatherer.gather();
        }
    }

    std::unique_ptr<Solver::SolutionInfo> Solver::solveCopiedTrees(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>();

        auto make_extBattery1 = [this, &end, &sol]() -> std::vector<ExtSingle> {
            std::vector<ExtSingle> battery;
            battery.reserve(this->problemcopies.size());
            for (std::size_t k = 0; k < this->problemcopies.size(); ++k) {
                battery.emplace_back(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient, *sol->trees[k], end);
            }
            return battery;
        };

        auto make_extBattery2 = [this, &sol](const std::vector<TreePtr>& otherTrees) -> std::vector<ExtBidir> {
            std::vector<ExtBidir> battery;
            battery.reserve(this->problemcopies.size());
            for (std::size_t k = 0; k < this->problemcopies.size(); ++k) {
                battery.emplace_back(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient, *sol->trees[k], *otherTrees[k]);
            }
            return battery;
        };


        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees = copied::TreeConcreteLinked::make_trees(this->problemcopies, std::make_unique<Node>(start));
            std::vector<ExtSingle> battery = make_extBattery1();
            solveParallel(battery, this->parameters.Iterations_Max, this->parameters.reallignment_coeff, copied::Gatherer(sol->trees));
            sol->iterations = battery.front().getIterationsDone();
            sol->solution = ExtSingle::computeBestSolutionSequence(battery);
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            sol->trees = copied::TreeConcreteLinked::make_trees(this->problemcopies, std::make_unique<Node>(start));
            auto temp = copied::TreeConcreteLinked::make_trees(this->problemcopies, std::make_unique<Node>(end));
            copied::GathererBid gatherer(sol->trees, temp);
            std::vector<ExtBidir> battery = make_extBattery2(temp);
            {
                sol->trees.reserve(this->problemcopies.size() * 2);
                for (auto it = temp.begin(); it != temp.end(); ++it) {
                    sol->trees.emplace_back(std::move(*it));
                }
            }
            solveParallel(battery, this->parameters.Iterations_Max, this->parameters.reallignment_coeff, gatherer);
            sol->iterations = battery.front().getIterationsDone();
            sol->solution = ExtBidir::computeBestSolutionSequence(battery);
        }
        else {
            sol->trees = copied::TreeStarLinked::make_trees(this->problemcopies, std::make_unique<Node>(start));
            std::vector<ExtSingle> battery = make_extBattery1();
            solveParallel(battery, this->parameters.Iterations_Max, this->parameters.reallignment_coeff, copied::Gatherer(sol->trees));
            sol->iterations = battery.front().getIterationsDone();
            sol->solution = ExtSingle::computeBestSolutionSequence(battery);
        }
        return sol;
    }
}