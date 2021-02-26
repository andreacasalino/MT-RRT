/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <TreeStar.h>

namespace mt {
    TreeStar::TreeStar(std::unique_ptr<TreeConcrete> wrapped)
        : TreeDecorator(std::move(wrapped)) {
    }

    void TreeStar::doRewire(Node* n) {
        if (nullptr == n) return;
        auto rew = static_cast<TreeConcrete*>(this->wrapped.get())->computeRewirds(*n);
        for (auto it = rew.begin(); it != rew.end(); ++it) {
            it->involved.setFather(&it->newFather, it->newCostFromFather);
        }
    }

    Node* TreeStar::extendRandom() {
        auto temp = this->wrapped->extendRandom();
        this->doRewire(temp);
        return temp;
    };

    std::pair<Node*, bool> TreeStar::extendDeterministic(const NodeState& target) {
        auto temp = this->wrapped->extendDeterministic(target);
        this->doRewire(temp.first);
        return temp;
    };
}