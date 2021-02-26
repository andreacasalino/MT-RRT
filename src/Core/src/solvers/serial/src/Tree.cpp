/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../Tree.h"

namespace mt::serial {
    TreeStar::TreeStar(Problem& problem, NodePtr root)
        : TreeConcrete(problem, std::move(root)) {
    }

    void TreeStar::doRewire(Node* n) {
        if (nullptr == n) return;
        auto rew = this->computeRewirds(*n);
        for (auto it = rew.begin(); it != rew.end(); ++it) {
            it->involved.setFather(&it->newFather, it->newCostFromFather);
        }
    }

    Node* TreeStar::extendRandom() {
        auto temp = this->TreeConcrete::extendRandom();
        this->doRewire(temp);
        return temp;
    };

    std::pair<Node*, bool> TreeStar::extendDeterministic(const NodeState& target) {
        auto temp = this->TreeConcrete::extendDeterministic(target);
        this->doRewire(temp.first);
        return temp;
    };
}