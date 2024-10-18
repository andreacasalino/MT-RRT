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
struct BidirSolution {
  std::vector<std::vector<float>> materialize() const;

  float cost() const;

  const Node *byPassFront;
  const Node *byPassBack;
  float cost2Back;
};

class ExtenderBidirectional : public ExtenderBase {
public:
  using SolutionT = BidirSolution;

  TreeHandlerPtr front_handler;
  TreeHandlerPtr back_handler;

  ExtenderBidirectional(TreeHandlerPtr front, TreeHandlerPtr back);

  void search_iteration(Solutions<BidirSolution>& solutions);

  std::vector<TreeHandlerPtr> dumpTrees() {
    std::vector<TreeHandlerPtr> res;
    res.emplace_back(std::move(front_handler));
    res.emplace_back(std::move(back_handler));
    return res;
  }

  const Parameters& parameters() const {
    return front_handler->parameters;
  }

private:
  bool extension_state = false; // if true master is front, slave is back
};
} // namespace mt_rrt
