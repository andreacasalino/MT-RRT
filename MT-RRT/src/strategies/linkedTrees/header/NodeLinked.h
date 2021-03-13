/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_NODE_LINKED_H
#define MT_RRT_NODE_LINKED_H

#include <Node.h>
#include <utility>

namespace mt::solver::linked {
    std::vector<NodePtr> make_copies(Node& node, const std::size_t& copiesNumber);

    class NodeLinked : public Node {
    public:
        NodeLinked(NodeLinked&&) = delete;

        static std::vector<std::unique_ptr<NodeLinked>> make_roots(Node& node, const std::size_t& threadsNumber);

        static std::vector<std::unique_ptr<NodeLinked>> make_linked(Node& node);

        inline const std::vector<NodeLinked*>& getLinked() const { return this->linkedNodes; };

        class NodeLinkedFactory;

    private:
        NodeLinked(const NodeState& state);

        static void link(const std::vector<std::unique_ptr<NodeLinked>>& group);

        std::vector<NodeLinked*> linkedNodes;
    };
}

#endif