/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../../Commons.h"
#include <strategies/LinkedTreesStrategy.h>
#include "header/TreeContainer.h"
#include <omp.h>
#include <atomic>

namespace mt::solver {
    template<typename E, typename Gather>
    void solveParallel(std::vector<E>& battery, const std::size_t& iterations, const double& reallCoeff, const Gather& gatherer) {
        std::atomic_bool life = true;
        mt::sampling::SeedFactory::resetSeeds();
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
                int ttt = omp_get_thread_num();
                gatherer();
            }
        }
        gatherer();
    }

    std::unique_ptr<SolutionInfo> LinkedTreesStrategy::solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>();

        auto make_extBattery1 = [this, &end](const std::vector<linked::TreeLinked*>& treeBattery) -> std::vector<ExtSingle> {           
            std::vector<ExtSingle> battery;            
            battery.reserve(this->solverData->problemsBattery.size());
            for (std::size_t k = 0; k < this->solverData->problemsBattery.size(); ++k) {
                battery.emplace_back(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient.get(), *treeBattery[k], end);
            }
            return battery;
        };

        auto make_extBattery2 = [this](const std::vector<linked::TreeLinked*>& treeBattery, const std::vector<linked::TreeLinked*>& otherBattery) -> std::vector<ExtBidir> {
            std::vector<ExtBidir> battery;
            battery.reserve(this->solverData->problemsBattery.size());
            for (std::size_t k = 0; k < this->solverData->problemsBattery.size(); ++k) {
                battery.emplace_back(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient.get(), *treeBattery[k], *otherBattery[k]);
            }
            return battery;
        };

        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<linked::TreeLinkedContainer>(std::make_unique<Node>(start),  this->solverData->problemsBattery) );
            std::vector<ExtSingle> battery = make_extBattery1(dynamic_cast<linked::TreeLinkedContainer*>(sol->trees.front().get())->getAsBattery() );
            solveParallel(battery, this->parameters.Iterations_Max.get(), this->reallignmentCoeff.get(), [&sol](){ dynamic_cast<linked::TreeLinkedContainer*>(sol->trees.front().get())->doGather(); });
            fillSolutionInfo(*sol, this->parameters, battery);
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<linked::TreeLinkedContainer>(std::make_unique<Node>(start),  this->solverData->problemsBattery) );
            sol->trees.emplace_back(std::make_unique<linked::TreeLinkedContainer>(std::make_unique<Node>(end),  this->solverData->problemsBattery) );
            std::vector<ExtBidir> battery = make_extBattery2(dynamic_cast<linked::TreeLinkedContainer*>(sol->trees.front().get())->getAsBattery(),
                                                             dynamic_cast<linked::TreeLinkedContainer*>(sol->trees.back().get())->getAsBattery() );
            solveParallel(battery, this->parameters.Iterations_Max.get(), this->reallignmentCoeff.get(), [&sol](){ 
                dynamic_cast<linked::TreeLinkedContainer*>(sol->trees.front().get())->doGather();  
                dynamic_cast<linked::TreeLinkedContainer*>(sol->trees.back().get())->doGather(); 
                });
            fillSolutionInfo(*sol, this->parameters, battery); 
        }
        else {
            sol->trees.emplace_back(std::make_unique<linked::TreeStarLinkedContainer>(std::make_unique<Node>(start),  this->solverData->problemsBattery) );
            std::vector<ExtSingle> battery = make_extBattery1(dynamic_cast<linked::TreeStarLinkedContainer*>(sol->trees.front().get())->getAsBattery() );
            solveParallel(battery, this->parameters.Iterations_Max.get(), this->reallignmentCoeff.get(), [&sol](){ dynamic_cast<linked::TreeStarLinkedContainer*>(sol->trees.front().get())->doGather(); });
            fillSolutionInfo(*sol, this->parameters, battery);
        }
        return sol;
    }
}