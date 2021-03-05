/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../TreeSlave.h"

namespace mt::multiag {
    TreeSlave::TreeSlave(Problem& problem)
        : TreeConcrete(problem, std::make_unique<Node>(NodeState{0.f})) {
        this->nodes.clear();
    }

    void TreeSlave::add(NodePtr node) {
        if (this->originalRoot == node->getFather()) {
            node->setFather(this->originalRoot, node->getCostFromFather());
        }
        this->nodes.emplace_back(std::move(node));
    }
}