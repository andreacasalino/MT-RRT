/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../Commons.h"
#include <strategies/SerialStrategy.h>
#include <TreeStar.h>

namespace mt::solver {
    std::unique_ptr<SolutionInfo> SerialStrategy::solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>();
        Problem& prb = *this->solverData->problemsBattery.front().get();
        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees.emplace_back( std::make_unique<TreeCore>( std::make_unique<Node>(start), prb ));
            solveSingle(*sol, this->parameters, end);
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<TreeCore>(std::make_unique<Node>(start), prb ));
            sol->trees.emplace_back(std::make_unique<TreeCore>(std::make_unique<Node>(end), prb ));
            solveBidir(*sol, this->parameters);
        }
        else {
            sol->trees.emplace_back(std::make_unique<TreeStar<TreeCore>>(std::make_unique<Node>(start), prb));
            solveSingle(*sol, this->parameters, end);
        }
        return sol;
    }
}