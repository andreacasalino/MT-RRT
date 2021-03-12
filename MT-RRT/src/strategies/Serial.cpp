/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../Commons.h"
#include <strategies/Serial.h>
#include <TreeStar.h>

namespace mt::solver {
    std::unique_ptr<SolutionInfo> Serial::solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>();
        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees.emplace_back( std::make_unique<TreeCore>(*this->solverData->problemsBattery.front(), std::make_unique<Node>(start)) );
            solveSingle(*sol, this->parameters, end);
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<TreeCore>(*this->solverData->problemsBattery.front(), std::make_unique<Node>(start)));
            sol->trees.emplace_back(std::make_unique<TreeCore>(*this->solverData->problemsBattery.front(), std::make_unique<Node>(end)));
            solveBidir(*sol, this->parameters);
        }
        else {
            sol->trees.emplace_back(std::make_unique<TreeStarBasic>(*this->solverData->problemsBattery.front(), std::make_unique<Node>(start)));
            solveSingle(*sol, this->parameters, end);
        }
        return sol;
    }
}