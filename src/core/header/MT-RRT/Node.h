/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Error.h>
#include <MT-RRT/Limited.h>
#include <MT-RRT/ObjectPool.h>
#include <MT-RRT/View.h>

#include <memory>
#include <optional>
#include <vector>

namespace mt_rrt {
/**
 * @brief Used for representing a state  x \in \underline{\mathcal{X}},
 * Section 1.2 of the documentation.
 * This class is used internally to extend search trees. The user is not
 * expected to consume it.
 */
class Node {
public:
  virtual ~Node() = default;

  Node(const View &state);

  const View &state() const { return state_; }

  /**
   * @brief position of the parent node inside the owning Tree
   */
  const Node *getParent() const { return parent_; }

  float cost2Go() const { return cost2Go_.get(); }
  void setParent(const Node &parent, float cost2Go);

  /**
   * @return Computes the cost to get from the root to this node, see 1.2.
   * @throw when the root is not reached, cause loopy connections were made for
   * some reason.
   */
  float cost2Root() const;

protected:
  Node() = default;

  View state_;
  /**
   * @brief The cost to spend to go from the parent to this node
   */
  Positive<float> cost2Go_ = Positive<float>{0};
  const Node *parent_ = nullptr;

  static const inline std::size_t MAX_ITERATIONS_COST2ROOT_DEDUCTION =
      std::numeric_limits<std::size_t>::max();
};

template <typename NodeT> class NodesAllocatorT {
public:
  NodesAllocatorT() = default;

  NodeT &emplace_back(const View &state) {
    if (state.size == 0) {
      throw Error("Empty state not valid for describing node state");
    }
    float *state_cpy = statesPool_.emplace_back(state.data, state.size);
    return nodesPool_.emplace_back(View{state_cpy, state.size});
  }

private:
  ObjectPool<float> statesPool_;
  ObjectPool<NodeT> nodesPool_;
};

using NodesAllocator = NodesAllocatorT<Node>;

class NodeOwning : public Node {
public:
  NodeOwning(std::vector<float> &&state);

  NodeOwning(const Node &o) : NodeOwning{o.state().convert()} {
    setParent(*o.getParent(), o.cost2Go());
  }

private:
  std::vector<float> stateVec_;
};
} // namespace mt_rrt
