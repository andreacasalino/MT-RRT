/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Node.h>
#include <MT-RRT/ObjectPool.h>

#include <deque>
#include <memory>

namespace mt_rrt {
class Nodes {
public:
  Nodes() = default;

  Node &push(std::span<const float> to_add) {
    auto copied_view = statesPool_->push(to_add);
    return nodesPool_.emplace_back(copied_view);
  }

  const auto &getNodes() const { return nodesPool_; }

private:
  std::unique_ptr<ObjectPool<float>> statesPool_;
  std::deque<Node> nodesPool_;
};
} // namespace mt_rrt
