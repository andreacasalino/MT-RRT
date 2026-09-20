/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Node.h>

#include <cstring>

namespace mt_rrt {
Node::Node(std::span<const float> state) : data_{.state = state} {}

void Node::setParent(const Node &parent, const Positive &cost2Go) noexcept {
  data_.parent = &parent;
  data_.cost2Go = cost2Go;
}

float Node::cost2Root() const {
  float cost2Root = 0;
  size_t k = 0;
  for (const Node *att_node = this; att_node != nullptr;
       att_node = att_node->data_.parent, ++k) {
    if (std::numeric_limits<std::size_t>::max() == k) {
      throw Error("Max number of iterations exceeded while computing cost to "
                  "go: a loop was generated inside a tree");
    }
    cost2Root += att_node->data_.cost2Go.get();
  }
  return cost2Root;
};

namespace detail {
NodeOwningStorage::NodeOwningStorage(std::vector<float> allocated_state)
    : storage_{std::move(allocated_state)} {}
} // namespace detail

NodeOwning::NodeOwning(std::vector<float> state)
    : detail::NodeOwningStorage{std::move(state)}, Node{std::span<const float>{
                                                       storage_.begin(),
                                                       storage_.end()}} {}
} // namespace mt_rrt
