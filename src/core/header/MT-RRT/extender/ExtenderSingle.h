/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/extender/Types.h>
#include <MT-RRT/TreeHandler.h>
#include <MT-RRT/Solution.h>

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

  void search_iteration(Solutions<SimpleSolution>& solutions);
  
  std::vector<TreeHandlerPtr> dumpTrees() {
    std::vector<TreeHandlerPtr> res;
    res.emplace_back(std::move(tree_handler));
    return res;
  }

  const Parameters& parameters() const {
    return tree_handler->parameters;
  }
};
} // namespace mt_rrt
