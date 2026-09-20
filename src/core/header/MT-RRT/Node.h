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

#include <deque>
#include <memory>
#include <optional>
#include <span>
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
  Node(std::span<const float> state);

  const auto &data() const { return data_; }

  void setParent(const Node &parent, const Positive &cost2Go) noexcept;

  /**
   * @return Computes the cost to get from the root to this node, see 1.2.
   * @throw when the root is not reached, cause loopy connections were made for
   * some reason.
   */
  float cost2Root() const;

  struct Data {
    std::span<const float> state;
    /**
     * @brief The cost to spend to go from the parent to this node
     */
    Positive cost2Go{0};
    const Node *parent{nullptr};
  };

protected:
  Data data_;
};

namespace detail {
class NodeOwningStorage {
protected:
  NodeOwningStorage(std::vector<float> allocated_state);

  std::vector<float> storage_;
};
} // namespace detail

class NodeOwning : public detail::NodeOwningStorage, public Node {
public:
  NodeOwning(std::vector<float> state);
};

template <typename NodeT> class Nodes {
public:
  Nodes() = default;

  NodeT &push(std::span<const float> to_add) {
    auto copied_view = statesPool_.push(to_add);
    return nodesPool_.emplace_back(copied_view);
  }

  const auto &getNodes() const { return nodesPool_; }

private:
  ObjectPool<float> statesPool_;
  std::deque<NodeT> nodesPool_;
};
} // namespace mt_rrt
