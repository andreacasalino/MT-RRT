/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Limited.h>

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

private:
  Data data_;
};

namespace detail {
class NodeOwningStorage {
protected:
  NodeOwningStorage(std::vector<float> allocated_state);

private:
  std::vector<float> storage_;
};
} // namespace detail

class NodeOwning : public detail::NodeOwningStorage, public Node {
public:
  NodeOwning(std::vector<float> state);
};
} // namespace mt_rrt
