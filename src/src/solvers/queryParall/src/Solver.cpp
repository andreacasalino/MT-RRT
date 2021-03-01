/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Solver.h>
#include <ExtenderSingle.h>
#include <ExtenderBidir.h>
#include "../TreeQParall.h"
#include <TreeStar.h>

namespace mt {
    std::unique_ptr<Solver::SolutionInfo> Solver::solveQueryParall(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>();
        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<qpar::TreeQPar>(this->problemcopies, std::make_unique<Node>(start)));
            static_cast<qpar::TreeQPar*>(sol->trees.back().get())->open();
            ExtSingle ext(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient, *sol->trees.front(), end);
            ext.extend(this->parameters.Iterations_Max);
            sol->iterations = ext.getIterationsDone();
            sol->solution = ext.computeBestSolutionSequence();
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<qpar::TreeQPar>(this->problemcopies, std::make_unique<Node>(start)));
            sol->trees.emplace_back(std::make_unique<qpar::TreeQPar>(static_cast<const qpar::TreeQPar&>(*sol->trees.back().get()), std::make_unique<Node>(end)));
            static_cast<qpar::TreeQPar*>(sol->trees.back().get())->open();
            ExtBidir ext(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient, *sol->trees.front(), *sol->trees.back());
            ext.extend(this->parameters.Iterations_Max);
            sol->iterations = ext.getIterationsDone();
            sol->solution = ext.computeBestSolutionSequence();
        }
        else {
            sol->trees.emplace_back(std::make_unique<TreeStar>(
                std::make_unique<qpar::TreeQPar>(this->problemcopies, std::make_unique<Node>(start))
                ));
            static_cast<TreeStar*>(sol->trees.back().get())->getT<qpar::TreeQPar>()->open();
            ExtSingle ext(this->parameters.Cumulate_sol, this->parameters.Deterministic_coefficient, *sol->trees.front(), end);
            ext.extend(this->parameters.Iterations_Max);
            sol->iterations = ext.getIterationsDone();
            sol->solution = ext.computeBestSolutionSequence();
        }
        static_cast<qpar::TreeQPar*>(sol->trees.back().get())->close();
        return sol;
    }
}