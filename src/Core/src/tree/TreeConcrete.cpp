/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "TreeConcrete.h"
#include <Error.h>
#include <algorithm>

namespace mt::solver::tree {
    TreeConcrete::TreeConcrete(problem::Problem& problem, NodePtr root)
        : problem(problem) {
        if (nullptr == root) {
            throw Error("null root is impossible for TreeConcrete");
        }
        this->nodes.emplace_back(std::move(root));
    }

    const Node* TreeConcrete::extendRandom() {
        return this->extendDeterministic(this->problem.randomState()).first;
    }

    std::pair<const Node*, bool> TreeConcrete::extendDeterministic(const NodeState& target) {
        Node* nearest = this->nearestNeighbour(target);
        bool temp;
        NodePtr ext = this->problem.steer(*nearest, target, temp);
        if (nullptr == ext) return {nullptr, false};
        this->nodes.emplace_back(std::move(ext));
        return { this->nodes.back().get(), temp };
    }

    Node* TreeConcrete::nearestNeighbour(const NodeState& state) {
        auto it = this->nodes.begin();
        Node* nearest = it->get();
        float nearestCost = this->problem.cost2Go(nearest->getState(), state, true), temp;
        ++it;
        std::for_each(it, this->nodes.end(), [&, this](NodePtr& n) {
            temp = this->problem.cost2Go(n->getState(), state, true);
            if (temp < nearestCost) {
                nearestCost = temp;
                nearest = n.get();
            }
        });
        return nearest;
    }
}