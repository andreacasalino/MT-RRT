/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-carpet/Limited.h>

#include <list>
#include <memory>
#include <vector>

namespace mt_rrt {
using State = std::vector<float>;

class Node;
struct NodeFatherInfo {
  Node *father = nullptr;
  float cost_from_father = 0;
};

/** @brief Used for representing a state  x \in \underline{\mathcal{X}},
 * Section 1.2 of the documentation.
 */
class Node {
public:
  /** @param the values inside the vector respresenting this state
   *  @throw when passing an empty state
   */
  Node(const State &state);

  virtual ~Node() = default;

  Node(const Node &) = delete;
  Node &operator=(const Node &) = delete;

  Node(Node &&o);
  Node &operator=(Node &&) = delete;

  /** @return the state describing this node
   */
  inline const State &getState() const { return this->state; };

  virtual NodeFatherInfo getFatherInfo() const {
    return NodeFatherInfo{father, costFromFather.get()};
  };

  virtual void setFatherInfo(const NodeFatherInfo &info = NodeFatherInfo{});

  /** @return Computes the cost to get from the root to this node, see 1.2.
   *  @throw when the root is not reached, cause loopy connections were made
   */
  float cost2Root() const;

private:
  State state;
  Node *father;
  Positive<float> costFromFather;
};

using NodePtr = std::shared_ptr<Node>;

NodePtr make_root(const State &state);

using Tree = std::list<NodePtr>;

using TreePtr = std::shared_ptr<Tree>;
} // namespace mt_rrt
