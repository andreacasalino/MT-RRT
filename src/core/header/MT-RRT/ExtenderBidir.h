/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Extender.h>

namespace mt_rrt {
class BidirSolution : public Solution {
public:
  BidirSolution(const Node *byPassFront, const Node *byPassBack,
                float cost2Back)
      : byPassFront{byPassFront}, byPassBack{byPassBack}, cost2Back{
                                                              cost2Back} {};

  std::vector<std::vector<float>> getSequence() const final;

  float cost() const final;

  const Node *byPassFront;
  const Node *byPassBack;
  float cost2Back;
};

class ExtenderBidirectional : public Extender {
public:
  TreeHandlerPtr front_handler;
  TreeHandlerPtr back_handler;

  ExtenderBidirectional(TreeHandlerPtr front, TreeHandlerPtr back);

  std::vector<TreeHandlerPtr> dumpTrees() final {
    std::vector<TreeHandlerPtr> res;
    res.emplace_back(std::move(front_handler));
    res.emplace_back(std::move(back_handler));
    return res;
  };

protected:
  void search_iteration() final;

private:
  bool extension_state = false; // if true master is front, slave is back
};
} // namespace mt_rrt
