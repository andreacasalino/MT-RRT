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
    TreeStarLinked::TreeStarLinked(Problem& problem, NodePtr root)
        : TreeConcreteLinked(problem, std::move(root)) {
    }

    void TreeStarLinked::add(NodePtr node) {
        if (nullptr == node) return;
        auto group = NodeLinked::make_linked(*node);
        std::size_t thId = static_cast<std::size_t>(omp_get_thread_num());
        auto itO = this->ListLinked<NodePtr>::outgoings.begin();
        for (std::size_t k = 0; k < group.size(); ++k) {
            if (thId == k) {
                this->TreeConcrete::add(std::move(group[k]));
            }
            else {
                (*itO)->emplace_back(std::move(group[k]));
                ++itO;
            }
        }
    }

    void TreeStarLinked::gather() {
        this->TreeConcreteLinked::gather();
        for (auto in = this->ListLinked<TreeConcrete::Rewird>::incomings.begin(); in != this->ListLinked<TreeConcrete::Rewird>::incomings.end(); ++in) {
            for (auto it = in->begin(); it != in->end(); ++it) {
                it->involved.setFather(&it->newFather, it->newCostFromFather);
            }
            in->clear();
        }
    }

    std::pair<NodePtr, bool> TreeStarLinked::extend(const NodeState& target) {
        auto temp = this->TreeConcrete::extend(target);
        if (nullptr == temp.first) return temp;
        if (temp.second) return temp; // no rewire on target node
        auto rew = this->computeRewirds(*temp.first.get(), this->getDelimiter());

        std::size_t k;
        for (auto itR = rew.begin(); itR != rew.end(); ++itR) {
            const std::vector<NodeLinked*>& involvedCopies = static_cast<NodeLinked&>(itR->involved).getLinked();
            const std::vector<NodeLinked*>& fatherCopies = static_cast<NodeLinked&>(itR->newFather).getLinked();
            auto itO = this->ListLinked<TreeConcrete::Rewird>::outgoings.begin();
            for (k = 0; k < involvedCopies.size(); ++k) {
                (*itO)->emplace_back(*involvedCopies[k], *fatherCopies[k], itR->newCostFromFather);
                ++itO;
            }
        }

        for (auto it = rew.begin(); it != rew.end(); ++it) {
            it->involved.setFather(&it->newFather, it->newCostFromFather);
        }
        return temp;
    }

    std::vector<TreePtr> TreeStarLinked::make_trees(const std::vector<ProblemPtr>& problems, NodePtr root) {
        std::vector<TreePtr> group;
        std::vector<ListLinked<NodePtr>*> groupPtr;
        std::vector<ListLinked<TreeConcrete::Rewird>*> groupPtr2;
        group.reserve(problems.size());
        groupPtr.reserve(problems.size());
        groupPtr2.reserve(problems.size());
        auto roots = NodeLinked::make_roots(*root, problems.size());
        for (std::size_t k = 0; k < problems.size(); ++k) {
            auto temp = new TreeStarLinked(*problems[k], std::move(roots[k]));
            group.emplace_back(temp);
            groupPtr.emplace_back(temp);
            groupPtr2.emplace_back(temp);
        }
        ListLinked<NodePtr>::link(groupPtr);
        ListLinked<TreeConcrete::Rewird>::link(groupPtr2);
        return group;
    }
}