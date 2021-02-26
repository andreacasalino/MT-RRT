/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Solver.h>
#include <ExtenderSingle.h>
#include <ExtenderBidir.h>
#include "header/TreeStar.h"

namespace mt {
    std::unique_ptr<Solver::SolutionInfo> Solver::solveSerial(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>();
        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees.emplace_back( std::make_unique<TreeConcrete>(*this->problemcopies[0], std::make_unique<Node>(start)) );
            ExtSingle ext(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient, *sol->trees.front(), end);
            ext.extend(this->parameters.Iterations_Max);
            sol->iterations = ext.getIterationsDone();
            sol->solution = ext.computeBestSolutionSequence();
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<TreeConcrete>(*this->problemcopies[0], std::make_unique<Node>(start)));
            sol->trees.emplace_back(std::make_unique<TreeConcrete>(*this->problemcopies[0], std::make_unique<Node>(end)));
            ExtBidir ext(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient, *sol->trees.front(), *sol->trees.back());
            ext.extend(this->parameters.Iterations_Max);
            sol->iterations = ext.getIterationsDone();
            sol->solution = ext.computeBestSolutionSequence();
        }
        else {
            sol->trees.emplace_back(std::make_unique<serial::TreeStar>(*this->problemcopies[0], std::make_unique<Node>(start)));
            ExtSingle ext(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient, *sol->trees.front(), end);
            ext.extend(this->parameters.Iterations_Max);
            sol->iterations = ext.getIterationsDone();
            sol->solution = ext.computeBestSolutionSequence();
        }
        return sol;
    }
}