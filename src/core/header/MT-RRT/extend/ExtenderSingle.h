/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/extend/Extend.h>

namespace mt_rrt {
template <typename T, Connector C> class ExtenderSingle : public Extender {
public:
  std::vector<float> target;
  TreeHandlerPtr tree_handler;

  ExtenderSingle(T tree, C &conn, std::span<const float> target);

  Nodes extractNodes();

  void extend();

  struct Solution {
    const Node *byPassNode;
    float cost2Target;
  };

  auto target() const { return target_; }

  const auto &solutions() const { return solutions_; }

private:
  C &connector_;
  std::span<const float> target_;
  T tree_;

  std::vector<Solution> solutions_;
};
} // namespace mt_rrt
