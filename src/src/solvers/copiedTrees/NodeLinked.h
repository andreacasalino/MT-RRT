/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_NODE_LINKED_H
#define MT_RRT_NODE_LINKED_H

#include <Node.h>

namespace mt::copied {
    class NodeLinked : public Node {
    public:
        NodeLinked(NodeLinked&&) = delete;

        static std::vector<NodePtr> make_copies(Node& node, const std::size_t& threadsNumber);

        static std::vector<NodePtr> make_roots(const NodeState& state, const std::size_t& threadsNumber);

        static std::vector<NodePtr> make_linked(Node&& node);

        inline const std::vector<Node*>& getLinked() const { return this->linkedNodes; };

        class NodeLinkedFactory;

    private:
        NodeLinked(const NodeState& state);
        NodeLinked(Node&& o);

        static void link(const std::vector<NodePtr>& group);

        std::vector<Node*> linkedNodes;
    };
}

#endif