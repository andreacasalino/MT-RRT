/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../TreeStarLinked.h"
#include "../NodeLinked.h"
#include <omp.h>

namespace mt::copied {
    void TreeStarLinked::add(NodePtr node) {
        auto group = NodeLinked::make_linked(std::move(*node));
        std::size_t thId = static_cast<std::size_t>(omp_get_thread_num());
        for (std::size_t k = 0; k < this->TreeConcreteLinked::outgoings.size(); ++k) {
            if (thId == k) {
                this->TreeConcrete::add(std::move(group[k]));

            }
            else {
                this->TreeConcreteLinked::outgoings[k]->emplace_back(std::move(group[k]));
            }
        }
    }

    void TreeStarLinked::gather() {
        this->TreeConcreteLinked::gather();

        for (auto it = this->ListLinked<TreeConcrete::Rewird>::incomings.begin(); it != this->ListLinked<TreeConcrete::Rewird>::incomings.end(); ++it) {
            for (auto itt = it->begin(); itt != it->end(); ++it) {
                itt->involved.setFather(&itt->newFather , itt->newCostFromFather);
            }
            it->clear();
        }
    }

    std::pair<NodePtr, bool> TreeStarLinked::extend(const NodeState& target) {
        auto temp = this->TreeConcrete::extend(target);
        if (nullptr == temp.first) return temp;
        if (temp.second) return temp; // no rewire on target node
        auto rew = this->computeRewirds(*temp.first.get(), this->getDelimiter());

        std::size_t k;
        for (auto itR = rew.begin(); itR != rew.end(); ++itR) {
            const std::vector<Node*>& involvedCopies = static_cast<NodeLinked&>(itR->involved).getLinked();
            const std::vector<Node*>& fatherCopies = static_cast<NodeLinked&>(itR->newFather).getLinked();
            for (k = 0; k < involvedCopies.size(); ++k) {
                this->ListLinked<TreeConcrete::Rewird>::outgoings[k]->emplace_back(*involvedCopies[k], *fatherCopies[k], itR->newCostFromFather);
            }
        }

        for (auto it = rew.begin(); it != rew.end(); ++it) {
            it->involved.setFather(&it->newFather, it->newCostFromFather);
        }
        return temp;
    }
}