/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Extender.h>

namespace mt_rrt {
class SimpleSolution : public Solution {
public:
  SimpleSolution(const Node *byPassNode, float cost2Target, const View &target)
      : byPassNode{byPassNode}, cost2Target(cost2Target), target(target){};

  std::vector<std::vector<float>> getSequence() const final;

  float cost() const final;

  const Node *byPassNode;
  float cost2Target;
  View target;
};

class ExtenderSingle : public Extender {
public:
  std::vector<float> target;
  TreeHandlerPtr tree_handler;

  ExtenderSingle(TreeHandlerPtr handler, const std::vector<float> &target);

  std::vector<TreeHandlerPtr> dumpTrees() final {
    std::vector<TreeHandlerPtr> res;
    res.emplace_back(std::move(tree_handler));
    return res;
  }

protected:
  void search_iteration() final;
};
} // namespace mt_rrt
