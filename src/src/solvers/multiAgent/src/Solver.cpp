/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Solver.h>
#include <ExtenderSingle.h>
#include <ExtenderBidir.h>
#include "../TreeMaster.h"
#include "../TreeStarMaster.h"
#include <omp.h>
#include <atomic>
#include <Error.h>
#include "../Commons.h"

namespace mt {
    template<typename E>
    void solveParallel(std::vector<E>& battery, const std::size_t& iterations, const double& reallCoeff, multiag::TreeSlave::TreeMaster* master) {
        std::atomic_bool life = true;
        std::size_t Threads = battery.size();
        std::size_t Batch_size = computeBatchSize(iterations, reallCoeff, Threads);

#pragma omp parallel \
num_threads(static_cast<int>(Threads))
        {
            int thId = omp_get_thread_num();
            E* Solver_to_use = &battery[thId];
            if (0 == thId) {
                auto solutionfound = [&battery]() {
                    for (auto it = battery.begin(); it != battery.end(); ++it) {
                        if (!it->getSolutions().empty()) {
                            return true;
                        }
                    }
                    return false;
                };

                for (size_t k = 0; k < iterations; k += Batch_size * Threads) {
                    if (solutionfound() && !Solver_to_use->isCumulating()) {
                        break;
                    }
                    master->dispatch();
#pragma omp barrier
                    Solver_to_use->extend(Batch_size);
#pragma omp barrier
                    master->gather();
                }
                life = false;
#pragma omp barrier
            }
            else {
                while (true) {
#pragma omp barrier
                    if (!life) break;
                    Solver_to_use->extend(Batch_size);
#pragma omp barrier
                }
            }
        }
    }

    std::unique_ptr<Solver::SolutionInfo> Solver::solveMultiAgent(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>();

        auto make_extBattery = [this, &end, &sol]() -> std::vector<ExtSingle> {
            std::vector<ExtSingle> battery;
            multiag::TreeSlave::TreeMaster* trPtr = dynamic_cast<multiag::TreeSlave::TreeMaster*>(sol->trees.back().get());
            battery.reserve(this->problemcopies.size());
            for (std::size_t k = 0; k < this->problemcopies.size(); ++k) {
                battery.emplace_back(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient, trPtr->getSlave(k), end);
            }
            return battery;
        };

        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<multiag::TreeSlave::TreeMaster>(this->problemcopies, std::make_unique<Node>(start)));
            std::vector<ExtSingle> battery = make_extBattery();
            solveParallel(battery, this->parameters.Iterations_Max, this->parameters.reallignment_coeff, static_cast<multiag::TreeSlave::TreeMaster*>(sol->trees.back().get()));
            fillSolutionInfo(*sol, this->parameters, battery);
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            throw Error("bidirectional startegy not possible for multi agent approach");
        }
        else {
            sol->trees.emplace_back(std::make_unique<multiag::TreeStarMaster>(this->problemcopies, std::make_unique<Node>(start)));
            std::vector<ExtSingle> battery = make_extBattery();
            solveParallel(battery, this->parameters.Iterations_Max, this->parameters.reallignment_coeff, dynamic_cast<multiag::TreeSlave::TreeMaster*>(sol->trees.back().get()));
            fillSolutionInfo(*sol, this->parameters, battery);
        }
        return sol;
    }
}