/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../TreeStarCritical.h"

namespace mt::shared {
    TreeStarCritical::TreeStarCritical(const std::vector<ProblemPtr>& problems, NodePtr root)
        : TreeConcreteCritical(problems, std::move(root)) {
    }

    std::pair<NodePtr, bool> TreeStarCritical::extend(const NodeState& target) {
        auto temp = this->TreeConcreteCritical::extend(target);
        if (nullptr == temp.first) return temp;
        if (temp.second) return temp; // no rewire on target node
        auto rew = this->computeRewirds(*temp.first.get(), this->getDelimiter());
        std::lock_guard<std::mutex> lock(this->mtx);
        for (auto it = rew.begin(); it != rew.end(); ++it) {
            it->involved.setFather(&it->newFather, it->newCostFromFather);
        }
        return temp;
    }
}