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

namespace mt {
    class Gatherer {
    public:
        Gatherer(const std::vector<TreePtr>& battery) {
            this->batteries.push_back(cast(battery));
        };

        Gatherer(const std::vector<TreePtr>& batteryA, const std::vector<TreePtr>& batteryB) {
            this->batteries.push_back(cast(batteryA));
            this->batteries.push_back(cast(batteryB));
        }

        void gather() const {
            int thId = omp_get_thread_num();
            for (auto it = this->batteries.begin(); it != this->batteries.end(); ++it) {
                (*it)[thId]->gather();
            }
        };

    private:
        static std::vector<copied::TreeConcreteLinked*> cast(const std::vector<TreePtr>& battery) {
            std::vector<copied::TreeConcreteLinked*> casted;
            casted.reserve(battery.size());
            for (auto it = battery.begin(); it != battery.cend(); ++it) {
                casted.push_back(dynamic_cast<copied::TreeConcreteLinked*>(it->get()));
            }
            return casted;
        };

        std::list<std::vector<copied::TreeConcreteLinked*>> batteries;
    };

    template<typename E>
    void solveParallel(std::vector<E>& battery, const std::size_t& iterations, const Gatherer& gatherer) {
        std::atomic_bool life = true;
        std::size_t Threads = battery.size();
        std::size_t Batch_size = (size_t)std::ceil(0.5f * static_cast<float>(iterations) / static_cast<float>(Threads));

#pragma omp parallel \
num_threads(static_cast<int>(Threads))
        {
            std::size_t Number_threads = static_cast<std::size_t>(omp_get_num_threads());
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
            solveParallel(battery, this->parameters.Iterations_Max, Gatherer(sol->trees));
            sol->iterations = battery.front().getIterationsDone();
            sol->solution = ExtSingle::computeBestSolutionSequence(battery);
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            sol->trees.reserve(this->problemcopies.size() * 2);
            sol->trees = copied::TreeConcreteLinked::make_trees(this->problemcopies, std::make_unique<Node>(start));
            auto temp = copied::TreeConcreteLinked::make_trees(this->problemcopies, std::make_unique<Node>(end));
            Gatherer gatherer(sol->trees, temp);
            std::vector<ExtBidir> battery = make_extBattery2(temp);
            {
                while (!temp.empty()) {
                    sol->trees.emplace_back(std::move(temp.front()));
                }
            }
            solveParallel(battery, this->parameters.Iterations_Max, gatherer);
            sol->iterations = battery.front().getIterationsDone();
            sol->solution = ExtBidir::computeBestSolutionSequence(battery);
        }
        else {
            sol->trees = copied::TreeStarLinked::make_trees(this->problemcopies, std::make_unique<Node>(start));
            std::vector<ExtSingle> battery = make_extBattery1();
            solveParallel(battery, this->parameters.Iterations_Max, Gatherer(sol->trees));
            sol->iterations = battery.front().getIterationsDone();
            sol->solution = ExtSingle::computeBestSolutionSequence(battery);
        }
        return sol;
    }
}