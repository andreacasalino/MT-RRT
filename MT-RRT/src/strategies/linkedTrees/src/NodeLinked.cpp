/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "../header/NodeLinked.h"
#include <omp.h>

namespace mt::solver::linked {
    std::vector<NodePtr> make_copies(Node& node) {
        std::vector<NodePtr> group;
        group.reserve(omp_get_num_threads() - 1);
        for (std::size_t k = 0; k < group.capacity(); ++k) {
            group.emplace_back(std::make_unique<Node>(node.getState()));
            group.back()->setFather(node.getFather(), node.getCostFromFather());
        }
        return group;
    }

    NodeLinked::NodeLinked(const NodeState& state)
        : Node(state) {
    }

    std::vector<std::unique_ptr<NodeLinked>> NodeLinked::make_roots(Node& node, const std::size_t& threadsNumber) {
        std::vector<std::unique_ptr<NodeLinked>> roots;
        roots.reserve(threadsNumber);
        for (std::size_t k = 0; k < threadsNumber; ++k) {
            roots.emplace_back(std::unique_ptr<NodeLinked>( new NodeLinked(node.getState()) ));
        }
        link(roots);
        return roots;
    }

    std::vector<std::unique_ptr<NodeLinked>> NodeLinked::make_linked(Node& node) {
        int threadsNumber = omp_get_num_threads();
        std::vector<std::unique_ptr<NodeLinked>> group;
        group.resize(threadsNumber);
        std::size_t thId = static_cast<std::size_t>(threadsNumber);
        const std::vector<NodeLinked*>& linkedFather = static_cast<NodeLinked*>(node.getFather())->getLinked();
        std::size_t c = 0;
        for (std::size_t k = 0; k < threadsNumber; ++k) {
            group[k] = std::unique_ptr<NodeLinked>(new NodeLinked(node.getState()));
            if(thId != k) {
                group[k]->setFather(linkedFather[c], node.getCostFromFather());
                ++c;
            }
        }
        group[thId]->setFather(node.getFather(), node.getCostFromFather());

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