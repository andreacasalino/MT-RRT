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

    std::pair<Node*, NodePtr> TreeStar::extend(const NodeState& target) {
        auto temp = this->wrapped->extend(target);
        if (nullptr == temp.second) return temp;
        auto rew = static_cast<TreeConcrete*>(this->wrapped.get())->computeRewirds(*temp.second.get());
        for (auto it = rew.begin(); it != rew.end(); ++it) {
            it->involved.setFather(&it->newFather, it->newCostFromFather);
        }
        return temp;
    }
}