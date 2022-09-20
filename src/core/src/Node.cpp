/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT-carpet/Error.h>
#include <MT-RRT-core/Node.h>

#include <limits>

namespace mt_rrt {
Node::Node(const State &state) : state(state) {
  if (this->state.empty()) {
    throw Error("empty state not valid for describing node state");
  }
  setFatherInfo();
}

Node::Node(Node &&o) : state(std::move(o.state)), father(o.father) {
  costFromFather.set(o.costFromFather.get());
}

void Node::setFatherInfo(const NodeFatherInfo &info) {
  this->father = info.father;
  this->costFromFather.set((this->father == nullptr) ? 0
                                                     : info.cost_from_father);
}

namespace {
constexpr std::size_t MAX_ITERATIONS = std::numeric_limits<std::size_t>::max();
}

float Node::cost2Root() const {
  float result = 0.f;
  const Node *att_node = this;
  size_t k = 0;
  while (att_node != nullptr) {
    const auto att_node_father = att_node->getFatherInfo();
    result += att_node_father.cost_from_father;
    att_node = att_node_father.father;
    if (MAX_ITERATIONS == ++k) {
      throw Error("Max number of iterations exceeded while computing cost to "
                  "go: a loop was generated inside a tree");
    }
  }
  return result;
};

NodePtr make_root(const State &state) { return std::make_shared<Node>(state); }
} // namespace mt_rrt
