/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>
#include <MT-RRT/Nodes.h>
#include <MT-RRT/NodesIteratorFromContainer.h>
#include <MT-RRT/Sampler.h>
#include <MT-RRT/Solution.h>
#include <MT-RRT/Tree.h>

#include <deque>
#include <optional>

namespace mt_rrt {
class TreeBase {
public:
  using iter_type =
      NodesIteratorFromContainer<typename std::deque<Node>::const_iterator>;

  TreeBase(std::span<const float> root);

  iter_type iter() const { return iter_type{nodes_.getNodes()}; }

  const auto &getNodes() const { return nodes_; }

  const Node *root() const { return &nodes_.getNodes().front(); }

  const Node *internalize(std::span<const float> state, const Node &parent,
                          const Positive &cost2Go) {
    auto &added = nodes_.push(state);
    added.setParent(parent, cost2Go);
    return &added;
  }

private:
  Nodes nodes_;
};
} // namespace mt_rrt
