/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Nodes.h>
#include <MT-RRT/NodesIterator.hxx>
#include <MT-RRT/Solution.h>
#include <MT-RRT/concepts/Connector.h>
#include <MT-RRT/concepts/Sampler.h>
#include <MT-RRT/concepts/Tree.h>

#include <deque>
#include <optional>

namespace mt_rrt {
class TreeBase {
public:
  using the_iter =
      NodesIteratorFromContainer<typename std::deque<Node>::const_iterator>;

  TreeBase(std::span<const float> root);

  the_iter iter() const { return the_iter{nodes_.getNodes()}; }

  const auto &getNodes() const { return nodes_; }

  const Node *internalize(std::span<const float> state, const Node &parent,
                          const Positive &cost2Go) {
    auto &added = nodes_.push(state);
    added.setParent(parent, cost2Go);
    return &added;
  }

  void apply(const Rewires &rew);

  auto &getDeterministicRegister() { return register_; }

private:
  Nodes nodes_;
  DeterministicSteerRegister register_;
};
} // namespace mt_rrt
