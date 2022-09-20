/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT-core/Trajectory.h>

namespace mt_rrt {
float Connector::minCost2GoConstrained(const State &start,
                                       const State &ending_node) const {
  float distance = minCost2Go(start, ending_node);
  if (COST_MAX == distance) {
    return distance;
  }

  auto traj = this->getTrajectory(start, ending_node);
  if (nullptr == traj) {
    throw Error{"found null trajectory but the min cost 2 go was not infinite"};
  }
  AdvanceInfo advInfo = AdvanceInfo::advanced;
  while (AdvanceInfo::advanced == advInfo) {
    advInfo = traj->advance();
    if (AdvanceInfo::blocked == advInfo) {
      return COST_MAX;
    }
  }
  return distance;
}

SteerIterations::SteerIterations() : LowerLimited<std::size_t>(1) {}

SteerIterations::SteerIterations(const std::size_t trials)
    : LowerLimited<std::size_t>(trials) {}

std::optional<SteerResult>
Connector::steer(Node &start, const State &trg,
                 const SteerIterations &trials) const {
  auto traj = getTrajectory(start.getState(), trg);
  if (nullptr == traj) {
    return std::nullopt;
  }
  State steered;
  float cost;
  bool trg_reached = false;
  AdvanceInfo info;
  for (std::size_t t = 0; t < trials.get(); ++t) {
    info = traj->advance();
    if (AdvanceInfo::blocked == info) {
      break;
    }
    steered = traj->getState();
    cost = traj->getCumulatedCost();
    if (AdvanceInfo::targetReached == info) {
      trg_reached = true;
      break;
    }
  }
  if (steered.empty()) {
    return std::nullopt;
  }
  SteerResult result =
      SteerResult{std::make_unique<Node>(steered), trg_reached};
  result.steered_node->setFatherInfo(NodeFatherInfo{&start, cost});
  return result;
}
} // namespace mt_rrt