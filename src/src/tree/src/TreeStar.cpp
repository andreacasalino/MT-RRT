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

    std::pair<NodePtr, bool> TreeStar::extend(const NodeState& target) {
        auto temp = this->get()->extend(target);
        if (nullptr == temp.first) return temp;
        if (temp.second) return temp; // no rewire on target node
        auto rew = this->get()->computeRewirds(*temp.first.get() , this->get()->getDelimiter());
        for (auto it = rew.begin(); it != rew.end(); ++it) {
            it->involved.setFather(&it->newFather, it->newCostFromFather);
        }
        return temp;
    }
}