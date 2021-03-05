/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Solver.h>
#include "../Commons.h"
#include <TreeStar.h>

namespace mt {
    std::unique_ptr<Solver::SolutionInfo> Solver::solveSerial(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>();
        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees.emplace_back( std::make_unique<TreeConcrete>(*this->problemcopies.front(), std::make_unique<Node>(start)) );
            solveSingle(*sol, this->parameters, end);
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<TreeConcrete>(*this->problemcopies.front(), std::make_unique<Node>(start)));
            sol->trees.emplace_back(std::make_unique<TreeConcrete>(*this->problemcopies.front(), std::make_unique<Node>(end)));
            solveBidir(*sol, this->parameters);
        }
        else {
            sol->trees.emplace_back(std::make_unique<TreeStar>(
                std::make_unique<TreeConcrete>(*this->problemcopies.front(), std::make_unique<Node>(start))
                ));
            solveSingle(*sol, this->parameters, end);
        }
        return sol;
    }
}