/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../../Commons.h"
#include <strategies/QueryParallStrategy.h>
#include "header/TreeQParall.h"
#include "header/TreeStarQParall.h"

namespace mt::solver {
    template<typename TreeT>
    inline TreeT* castTree(const TreePtr& tree) {
        return dynamic_cast<TreeT*>(tree.get());
    }

    std::unique_ptr<SolutionInfo> QueryParallStrategy::solve(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>(start, end);
        qpar::TreeQPar* poolRef = nullptr;
        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<qpar::TreeQPar>(std::make_unique<Node>(start), this->solverData->problemsBattery));
            poolRef = castTree<qpar::TreeQPar>(sol->trees.back());
            poolRef->open();
            solveSingle(*sol, this->parameters, end);
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<qpar::TreeQPar>(std::make_unique<Node>(start), this->solverData->problemsBattery));
            sol->trees.emplace_back(std::make_unique<qpar::TreeQPar>(std::make_unique<Node>(end), *castTree<qpar::TreeQPar>(sol->trees.back())));
            poolRef = castTree<qpar::TreeQPar>(sol->trees.back());
            poolRef->open();
            solveBidir(*sol, this->parameters);
        }
        else {
            sol->trees.emplace_back(std::make_unique<qpar::TreeStarQPar>(std::make_unique<Node>(start), this->solverData->problemsBattery));
            poolRef = castTree<qpar::TreeQPar>(sol->trees.back());
            poolRef->open();
            solveSingle(*sol, this->parameters, end);
        }
        poolRef->close();
        return sol;
    }
}