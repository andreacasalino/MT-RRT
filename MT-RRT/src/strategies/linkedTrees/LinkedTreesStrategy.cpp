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
                 Solver_to_use->extend(Batch_size);
                if (!Solver_to_use->getSolutions().empty() && (!Solver_to_use->isCumulating())) {
                    life = false;
                }
#pragma omp barrier
                gatherer();
                if (!life) break;
             }
        }
        gatherer();
    }

    std::unique_ptr<SolutionInfo> LinkedTreesStrategy::solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>();

        auto make_extBattery1 = [this, &end](const linked::TreeContainer& container) -> std::vector<ExtSingle> {           
            std::vector<ExtSingle> battery;            
            battery.reserve(container.size());
            for (std::size_t k = 0; k < container.size(); ++k) {
                battery.emplace_back(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient.get(), *container.getContained(k), end);
            }
            return battery;
        };

        auto make_extBattery2 = [this](const linked::TreeContainer& container, const linked::TreeContainer& container2) -> std::vector<ExtBidir> {
            std::vector<ExtBidir> battery;        
            battery.reserve(container.size());
            for (std::size_t k = 0; k < container.size(); ++k) {
                battery.emplace_back(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient.get(), *container.getContained(k), *container2.getContained(k));
            }
            return battery;
        };

        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<linked::TreeContainer>(std::make_unique<Node>(start),  this->solverData->problemsBattery) );
            std::vector<ExtSingle> battery = make_extBattery1( *dynamic_cast<const linked::TreeContainer*>(sol->trees.front().get()) );
            solveParallel(battery, this->parameters.Iterations_Max.get(), this->reallignmentCoeff.get(), [&sol](){ 
                dynamic_cast<linked::TreeContainer*>(sol->trees.front().get())->gather(); });
            fillSolutionInfo(*sol, this->parameters, battery);
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<linked::TreeContainer>(std::make_unique<Node>(start),  this->solverData->problemsBattery) );
            sol->trees.emplace_back(std::make_unique<linked::TreeContainer>(std::make_unique<Node>(end),  this->solverData->problemsBattery) );
            std::vector<ExtBidir> battery = make_extBattery2( *dynamic_cast<const linked::TreeContainer*>(sol->trees.front().get()),
                                                              *dynamic_cast<const linked::TreeContainer*>(sol->trees.back().get()) );
            solveParallel(battery, this->parameters.Iterations_Max.get(), this->reallignmentCoeff.get(), [&sol](){ 
                dynamic_cast<linked::TreeContainer*>(sol->trees.front().get())->gather();  
                dynamic_cast<linked::TreeContainer*>(sol->trees.back().get())->gather(); 
                });
            fillSolutionInfo(*sol, this->parameters, battery); 
        }
        else {
            sol->trees.emplace_back(std::make_unique<linked::TreeStarContainer>(std::make_unique<Node>(start),  this->solverData->problemsBattery) );
            std::vector<ExtSingle> battery = make_extBattery1( *dynamic_cast<const linked::TreeContainer*>(sol->trees.front().get()) );
            solveParallel(battery, this->parameters.Iterations_Max.get(), this->reallignmentCoeff.get(), [&sol](){ 
                dynamic_cast<linked::TreeContainer*>(sol->trees.front().get())->gather(); });
            fillSolutionInfo(*sol, this->parameters, battery);
        }
        return sol;
    }
}