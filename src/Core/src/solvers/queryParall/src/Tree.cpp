/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../Tree.h"

namespace mt::qpar {
    Tree::Tree(const std::vector<ProblemPtr>& problems, NodePtr root)
        : TreeConcrete(*problems.front(), std::move(root)) {
        this->problems.reserve(problems.size());
        for (std::size_t k = 0; k < problems.size(); ++k) {
            this->problems.push_back(problems[k].get());
        }
        this->pool = std::make_shared<Pool>();
    }

    Tree::Tree(const Tree& o, NodePtr root)
        : TreeConcrete(*o.problems.front(), std::move(root))
        , pool(o.pool) {
        this->problems = o.problems;
    }

    Node* Tree::nearestNeighbour(const NodeState& state) const {
        // todo
    }

    std::set<Node*> Tree::nearSet(Node& node) const {
        // todo
    }
}