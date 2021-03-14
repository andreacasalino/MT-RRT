/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/TreeLinked.h"
#include "../header/NodeLinked.h"
#include "../../ProblemBattery.h"

namespace mt::solver::linked {
    TreeLinked::TreeLinked(NodePtr root, Problem& problem)
        : TreeCore(std::move(root), problem) {
    }

    Node* TreeLinked::add(NodePtr node) {
        if (nullptr == node) return nullptr;
        auto group = make_copies(*node, this->ListLinked<NodePtr>::outgoings.size());
        Node* n = this->TreeCore::add(std::move(node));
        auto itG = group.begin();
        for (auto it = this->ListLinked<NodePtr>::outgoings.begin(); it != this->ListLinked<NodePtr>::outgoings.end(); ++it) {
            (*it)->emplace_back(std::move(*itG));
            ++itG;
        }
        return n;
    }

    void TreeLinked::gather() {
        this->ListLinked<NodePtr>::gather([this](NodePtr& n){ this->TreeCore::add(std::move(n)); });
    }

    std::vector<TreePtr> TreeLinked::make_trees(NodePtr root, const std::vector<ProblemPtr>& problems) {
        checkBattery(problems);
        std::vector<TreePtr> group;
        std::vector<ListLinked<NodePtr>*> groupPtr;
        group.reserve(problems.size());
        groupPtr.reserve(problems.size());
        auto roots = NodeLinked::make_roots(*root, problems.size());
        for (std::size_t k = 0; k < problems.size(); ++k) {
            auto temp = new TreeLinked(std::move(roots[k]), *problems[k]);
            group.emplace_back(temp);
            groupPtr.emplace_back(temp);
        }
        ListLinked<NodePtr>::link(groupPtr);
        return group;
    }
}
