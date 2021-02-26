/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Solver.h>
#include <ExtenderSingle.h>
#include <ExtenderBidir.h>
#include "../Tree.h"
#include <TreeStar.h>

namespace mt {
    std::unique_ptr<Solver::SolutionInfo> Solver::solveQueryParall(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>();
        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<qpar::Tree>(this->problemcopies, std::make_unique<Node>(start)));
            static_cast<qpar::Tree*>(sol->trees.back().get())->open();            
            ExtSingle ext(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient, *sol->trees.front(), end);
            ext.extend(this->parameters.Iterations_Max);
            sol->iterations = ext.getIterationsDone();
            sol->solution = ext.computeBestSolutionSequence();
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<qpar::Tree>(this->problemcopies, std::make_unique<Node>(start)));
            sol->trees.emplace_back(std::make_unique<qpar::Tree>(static_cast<const qpar::Tree&>(*sol->trees.back().get()), std::make_unique<Node>(end)));
            static_cast<qpar::Tree*>(sol->trees.back().get())->open();
            ExtBidir ext(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient, *sol->trees.front(), *sol->trees.back());
            ext.extend(this->parameters.Iterations_Max);
            sol->iterations = ext.getIterationsDone();
            sol->solution = ext.computeBestSolutionSequence();
        }
        else {
            sol->trees.emplace_back(std::make_unique<TreeStar>(
                std::make_unique<qpar::Tree>(this->problemcopies, std::make_unique<Node>(start))
                ));
            static_cast<TreeStar*>(sol->trees.back().get())->get<qpar::Tree>()->open();
            ExtSingle ext(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient, *sol->trees.front(), end);
            ext.extend(this->parameters.Iterations_Max);
            sol->iterations = ext.getIterationsDone();
            sol->solution = ext.computeBestSolutionSequence();
        }
        static_cast<qpar::Tree*>(sol->trees.back().get())->close();
        return sol;
    }
}