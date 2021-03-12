/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Solver.h>
#include "../Commons.h"
#include "../TreeQParall.h"
#include <TreeStar.h>

namespace mt {
    template<typename TreeT>
    inline TreeT* castTree(const TreePtr& tree) {
        return static_cast<TreeT*>(tree.get());
    }

    std::unique_ptr<Solver::SolutionInfo> Solver::solveQueryParall(const NodeState& start, const NodeState& end, const RRTStrategy& rrtStrategy) {
        auto sol = std::make_unique<SolutionInfo>();
        qpar::TreeQPar* poolRef = nullptr;
        if (RRTStrategy::Single == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<qpar::TreeQPar>(this->problemcopies, std::make_unique<Node>(start)));
            poolRef = castTree<qpar::TreeQPar>(sol->trees.back());
            poolRef->open();
            solveSingle(*sol, this->parameters, end);
        }
        else if (RRTStrategy::Bidir == rrtStrategy) {
            sol->trees.emplace_back(std::make_unique<qpar::TreeQPar>(this->problemcopies, std::make_unique<Node>(start)));
            sol->trees.emplace_back(std::make_unique<qpar::TreeQPar>(*castTree<qpar::TreeQPar>(sol->trees.back()), std::make_unique<Node>(end)));
            poolRef = castTree<qpar::TreeQPar>(sol->trees.back());
            poolRef->open();
            solveBidir(*sol, this->parameters);
        }
        else {
            sol->trees.emplace_back(std::make_unique<TreeStar>(
                std::make_unique<qpar::TreeQPar>(this->problemcopies, std::make_unique<Node>(start))
                ));            
            poolRef = castTree<TreeStar>(sol->trees.back())->getT<qpar::TreeQPar>();
            poolRef->open();
            solveSingle(*sol, this->parameters, end);
        }
        poolRef->close();
        return sol;
    }
}