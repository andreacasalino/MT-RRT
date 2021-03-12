/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../TreeConcreteLinked.h"
#include "../NodeLinked.h"

namespace mt::copied {
    TreeConcreteLinked::TreeConcreteLinked(Problem& problem, NodePtr root)
        : TreeConcrete(problem, std::move(root)) {
    }

    void TreeConcreteLinked::add(NodePtr node) {
        if (nullptr == node) return;
        auto group = make_copies(*node, this->ListLinked<NodePtr>::outgoings.size());
        this->TreeConcrete::add(std::move(node));
        auto itG = group.begin();
        for (auto it = this->ListLinked<NodePtr>::outgoings.begin(); it != this->ListLinked<NodePtr>::outgoings.end(); ++it) {
            (*it)->emplace_back(std::move(*itG));
            ++itG;
        }
    }

    void TreeConcreteLinked::gather() {
        for (auto in = this->ListLinked<NodePtr>::incomings.begin(); in != this->ListLinked<NodePtr>::incomings.end(); ++in) {
            for (auto it = in->begin(); it != in->end(); ++it) {
                this->TreeConcrete::add(std::move(*it));
            }
            in->clear();
        }
    }

    std::vector<TreePtr> TreeConcreteLinked::make_trees(const std::vector<ProblemPtr>& problems, NodePtr root) {
        std::vector<TreePtr> group;
        std::vector<ListLinked<NodePtr>*> groupPtr;
        group.reserve(problems.size());
        groupPtr.reserve(problems.size());
        auto roots = NodeLinked::make_roots(*root, problems.size());
        for (std::size_t k = 0; k < problems.size(); ++k) {
            auto temp = new TreeConcreteLinked(*problems[k], std::move(roots[k]));
            group.emplace_back(temp);
            groupPtr.emplace_back(temp);
        }
        ListLinked<NodePtr>::link(groupPtr);
        return group;
    }
}
