/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Extender.h>

namespace mt_rrt {
struct SimpleSolution {
  std::vector<std::vector<float>> materialize() const;

  float cost() const;

  const Node *byPassNode;
  float cost2Target;
  View target;
};

class ExtenderSingle : public ExtenderBase {
public:
  using SolutionT = SimpleSolution;

  std::vector<float> target;
  TreeHandlerPtr tree_handler;

  ExtenderSingle(TreeHandlerPtr handler, std::vector<float> &&target);

  void search_iteration();
};
} // namespace mt_rrt
