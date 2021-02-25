/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <solver/Solver.h>
#include <ExtenderSingle.h>
#include <ExtenderBidir.h>
#include "header/TreeStar.h"

namespace mt::solver {
    std::unique_ptr<Solver::SolutionInfo> Solver::serialStrategy(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>();
        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees.emplace_back( std::make_unique<tree::TreeConcrete>(*this->problemcopies[0], std::make_unique<Node>(start)) );
            extn::Single ext(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient, *sol->trees.front(), end);
            ext.extend(this->parameters.Iterations_Max);
            sol->iterations = ext.getIterationsDone();
            sol->solution = ext.computeBestSolutionSequence();
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<tree::TreeConcrete>(*this->problemcopies[0], std::make_unique<Node>(start)));
            sol->trees.emplace_back(std::make_unique<tree::TreeConcrete>(*this->problemcopies[0], std::make_unique<Node>(end)));
            extn::Bidir ext(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient, *sol->trees.front(), *sol->trees.back());
            ext.extend(this->parameters.Iterations_Max);
            sol->iterations = ext.getIterationsDone();
            sol->solution = ext.computeBestSolutionSequence();
        }
        else {
            sol->trees.emplace_back(std::make_unique<tree::TreeStar>(*this->problemcopies[0], std::make_unique<Node>(start)));
            extn::Single ext(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient, *sol->trees.front(), end);
            ext.extend(this->parameters.Iterations_Max);
            sol->iterations = ext.getIterationsDone();
            sol->solution = ext.computeBestSolutionSequence();
        }
        return sol;
    }
}