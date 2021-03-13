/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/TreeStarLinked.h"
#include "../header/NodeLinked.h"
#include "../../ProblemBattery.h"
#include <omp.h>

namespace mt::solver::linked {
    TreeStarLinked::TreeStarLinked(NodePtr root, Problem& problem)
        : TreeLinked(std::move(root), problem) {
    }

    Node* TreeStarLinked::add(NodePtr node) {
        if (nullptr == node) return nullptr;
        // dispatch linked copies
        auto group = NodeLinked::make_linked(*node);
        std::size_t thId = static_cast<std::size_t>(omp_get_thread_num());
        auto itO = this->ListLinked<NodePtr>::outgoings.begin();
        for (std::size_t k = 0; k < group.size(); ++k) {
            if (thId != k) {
                (*itO)->emplace_back(std::move(group[k]));
                ++itO;
            }
        }
        // compute and dispatch rewires
        auto rew = this->computeRewires(*group[thId].get());
        std::size_t k;
        for (auto itR = rew.begin(); itR != rew.end(); ++itR) {
            const std::vector<NodeLinked*>& involvedCopies = static_cast<NodeLinked&>(itR->involved).getLinked();
            const std::vector<NodeLinked*>& fatherCopies = static_cast<NodeLinked&>(itR->newFather).getLinked();
            auto itO = this->ListLinked<Rewire>::outgoings.begin();
            for (k = 0; k < involvedCopies.size(); ++k) {
                (*itO)->emplace_back(*involvedCopies[k], *fatherCopies[k], itR->newCostFromFather);
                ++itO;
            }
        }
        for (auto it = rew.begin(); it != rew.end(); ++it) {
            it->involved.setFather(&it->newFather, it->newCostFromFather);
        }
        // add node
        return this->TreeCore::add(std::move(group[thId]));
    }

    void TreeStarLinked::gather() {
        this->TreeLinked::gather();
        for (auto in = this->ListLinked<Rewire>::incomings.begin(); in != this->ListLinked<Rewire>::incomings.end(); ++in) {
            for (auto it = in->begin(); it != in->end(); ++it) {
                it->involved.setFather(&it->newFather, it->newCostFromFather);
            }
            in->clear();
        }
    }

    std::vector<TreePtr> TreeStarLinked::make_trees(NodePtr root, const std::vector<ProblemPtr>& problems) {
        checkBattery(problems);
        std::vector<TreePtr> group;
        std::vector<ListLinked<NodePtr>*> groupPtr;
        std::vector<ListLinked<Rewire>*> groupPtr2;
        group.reserve(problems.size());
        groupPtr.reserve(problems.size());
        groupPtr2.reserve(problems.size());
        auto roots = NodeLinked::make_roots(*root, problems.size());
        for (std::size_t k = 0; k < problems.size(); ++k) {
            auto temp = new TreeStarLinked(std::move(roots[k]), *problems[k]);
            group.emplace_back(temp);
            groupPtr.emplace_back(temp);
            groupPtr2.emplace_back(temp);
        }
        ListLinked<NodePtr>::link(groupPtr);
        ListLinked<Rewire>::link(groupPtr2);
        return group;
    }
}