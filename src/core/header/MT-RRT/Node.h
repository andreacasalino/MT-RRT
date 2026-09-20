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

  Node(std::span<const float> state);

  const auto &dat() const { return data_; }

  auto state() const { return state_; }

  void setParent(const Node &parent, float cost2Go);

  /**
   * @return Computes the cost to get from the root to this node, see 1.2.
   * @throw when the root is not reached, cause loopy connections were made for
   * some reason.
   */
  virtual float cost2Root() const;

  struct Data {
    std::span<const float> state_;
    /**
     * @brief The cost to spend to go from the parent to this node
     */
    Positive<float> cost2Go_ = Positive<float>{0};
    const Node *parent_{nullptr};
  };

protected:
  Data data_;
};

template <typename NodeT> class NodesT {
public:
  NodesT() = default;

  NodeT &emplace_back(std::span<const float> to_add) {
    auto copied_view = statesPool_.push(to_add);
    return nodesPool_.emplace_back(copied_view);
  }

  const auto &getNodes() const { return nodesPool_; }

private:
  ObjectPool<float> statesPool_;
  std::deque<NodeT> nodesPool_;
};

using Nodes = NodesT<Node>;

class NodeOwning : public Node {
public:
  NodeOwning(std::vector<float> state);

private:
  std::vector<float> allocated_state_;
};
} // namespace mt_rrt
