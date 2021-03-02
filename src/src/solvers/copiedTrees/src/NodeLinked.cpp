/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../NodeLinked.h"
#include <omp.h>

namespace mt::copied {
    std::vector<NodePtr> NodeLinked::make_copies(Node& node, const std::size_t& threadsNumber) {
        std::vector<NodePtr> group;
        group.reserve(threadsNumber - 1);
        for (std::size_t k = 1; k < threadsNumber; ++k) {
            group.emplace_back(std::make_unique<Node>(node.getState()));
            group.back()->setFather(node.getFather(), node.getCostFromFather());
        }
        return group;
    }

    NodeLinked::NodeLinked(const NodeState& state)
        : Node(state) {
    }

    std::vector<NodePtr> NodeLinked::make_roots(const NodeState& state, const std::size_t& threadsNumber) {
        std::vector<NodePtr> roots;
        roots.reserve(threadsNumber);
        for (std::size_t k = 0; k < threadsNumber; ++k) {
            roots.emplace_back(std::make_unique<NodeLinked>(state));
        }
        link(roots);
        return roots;
    }

    NodeLinked::NodeLinked(Node&& o)
        : NodeLinked(std::move(o)) {
    }

    std::vector<NodePtr> NodeLinked::make_linked(Node&& node) {
        std::size_t threadsNumber = static_cast<NodeLinked*>(&node)->getLinked().size() + 1;
        std::vector<NodePtr> group;
        group.resize(threadsNumber);
        std::size_t thId = static_cast<std::size_t>(omp_get_thread_num());
        std::size_t c = 0;
        for (std::size_t k = 0; k < threadsNumber; ++k) {
            if(thId != k) {
                group[k] = std::make_unique<NodeLinked>(node.getState());
                group[k]->setFather(static_cast<NodeLinked*>(&node)->getLinked()[c], node.getCostFromFather());
                ++c;
            }
        }
        group[thId] = std::make_unique<NodeLinked>(std::move(node));
        group[thId]->setFather(node.getFather(), node.getCostFromFather());
        link(group);
        return group;
    }

    void NodeLinked::link(const std::vector<NodePtr>& group) {
        std::size_t l;
        for (std::size_t k = 0; k < group.size(); ++k) {
            static_cast<NodeLinked*>(group[k].get())->linkedNodes.reserve(group.size() - 1);
            for (l = 0; l < group.size(); ++l) {
                if (l != k) {
                    static_cast<NodeLinked*>(group[k].get())->linkedNodes.push_back(group[l].get());
                }
            }
        }
    }
}