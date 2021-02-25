/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/TreeStar.h"

namespace mt::solver::tree {
    TreeStar::TreeStar(problem::Problem& problem, NodePtr root)
        : TreeConcrete(problem, std::move(root)) {
    }

    const Node* TreeStar::extendRandom() {
        auto temp = this->TreeConcrete::extendRandom();
        if (nullptr != temp) {
            auto rew = this->computeRewirds(*temp);

        }
        return temp;
    };

    std::pair<const Node*, bool> TreeStar::extendDeterministic(const NodeState& target) {

    };
}