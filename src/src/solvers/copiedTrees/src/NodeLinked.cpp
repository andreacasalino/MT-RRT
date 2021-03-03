/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../NodeLinked.h"
#include <omp.h>

namespace mt::copied {
    std::vector<NodePtr> make_copies(Node& node, const std::size_t& copiesNumber) {
        std::vector<NodePtr> group;
        group.reserve(copiesNumber);
        for (std::size_t k = 0; k < copiesNumber; ++k) {
            group.emplace_back(std::make_unique<Node>(node.getState()));
            group.back()->setFather(node.getFather(), node.getCostFromFather());
        }
        return group;
    }

    NodeLinked::NodeLinked(NodePtr node)
        : Node(std::move(*node)) {
    }

    NodeLinked::NodeLinked(const NodeState& state)
        : Node(state) {
    }

    std::vector<std::unique_ptr<NodeLinked>> NodeLinked::make_roots(NodePtr node, const std::size_t& threadsNumber) {
        std::vector<std::unique_ptr<NodeLinked>> roots;
        roots.reserve(threadsNumber);
        for (std::size_t k = 0; k < threadsNumber; ++k) {
            roots.emplace_back(std::unique_ptr<NodeLinked>( new NodeLinked(std::move(node)) ));
        }
        link(roots);
        return roots;
    }

    std::vector<std::unique_ptr<NodeLinked>> NodeLinked::make_linked(NodePtr node, const std::size_t& threadsNumber) {
        std::vector<std::unique_ptr<NodeLinked>> group;
        group.resize(threadsNumber);
        std::size_t thId = static_cast<std::size_t>(omp_get_thread_num());
        std::size_t c = 0;
        for (std::size_t k = 0; k < threadsNumber; ++k) {
            if(thId != k) {
                group[k] = std::unique_ptr<NodeLinked>( new NodeLinked(node->getState()) );
                group[k]->setFather(static_cast<NodeLinked*>(node->getFather())->getLinked()[c], node->getCostFromFather());
                ++c;
            }
        }
        group[thId] = std::unique_ptr<NodeLinked>( new NodeLinked(std::move(node)) );
        link(group);
        return group;
    }

    void NodeLinked::link(const std::vector<std::unique_ptr<NodeLinked>>& group) {
        std::size_t l;
        for (std::size_t k = 0; k < group.size(); ++k) {
            group[k]->linkedNodes.reserve(group.size() - 1);
            for (l = 0; l < group.size(); ++l) {
                if (l != k) {
                    group[k]->linkedNodes.push_back(group[l].get());
                }
            }
        }
    }
}