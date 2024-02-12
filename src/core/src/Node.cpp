/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Node.h>

#include <cstring>

namespace mt_rrt {
Node::Node(const View &state) : state_{state} {}

void Node::setParent(const Node &parent, float cost2Go) {
  parent_ = &parent;
  cost2Go_.set(cost2Go);
}

float Node::cost2Root() const {
  float cost2Root = 0;
  size_t k = 0;
  for (const Node *att_node = this; att_node != nullptr;
       att_node = att_node->getParent(), ++k) {
    if (MAX_ITERATIONS == k) {
      throw Error("Max number of iterations exceeded while computing cost to "
                  "go: a loop was generated inside a tree");
    }
    cost2Root += att_node->cost2Go();
  }
  return cost2Root;
};

NodeOwning::NodeOwning(std::vector<float> &&state)
    : stateVec_{std::forward<std::vector<float>>(state)} {
  state_ = View{stateVec_.data(), stateVec_.size()};
}
} // namespace mt_rrt
