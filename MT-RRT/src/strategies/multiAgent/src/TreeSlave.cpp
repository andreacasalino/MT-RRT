/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/TreeSlave.h"

namespace mt::solver::multiag {
    TreeSlave::TreeSlave(Problem& problem)
        : TreeCore(std::make_unique<Node>(NodeState{0.f}), problem) {
        this->nodes.clear();
    }

    Node* TreeSlave::add(NodePtr node) {
        if (nullptr == node) return nullptr;
        if (this->nodes.front().get() == node->getFather()) {
            node->setFather(this->originalRoot, node->getCostFromFather());
        }
        this->nodes.emplace_back(std::move(node));
        return this->nodes.back().get();
    }
}