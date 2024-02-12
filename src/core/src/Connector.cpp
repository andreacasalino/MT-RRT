/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Connector.h>

namespace mt_rrt {
SteerIterations::SteerIterations() : LowerLimited<std::size_t>(1) {}

SteerIterations::SteerIterations(const std::size_t trials)
    : LowerLimited<std::size_t>(trials) {}

float Connector::minCost2GoConstrained(const View &start,
                                       const View &ending_node) const {
  float distance = minCost2Go(start, ending_node);
  if (COST_MAX == distance) {
    return distance;
  }

  auto traj = this->getTrajectory(start, ending_node);
  if (nullptr == traj) {
    throw Error{"found null trajectory but the min cost 2 go was not infinite"};
  }
  Trajectory::AdvanceInfo advInfo = traj->advance();
  for (; advInfo == Trajectory::AdvanceInfo::advanced;
       advInfo = traj->advance()) {
  }
  return advInfo == Trajectory::AdvanceInfo::blocked ? COST_MAX : distance;
}

std::optional<Connector::SteerResult>
Connector::steer(const Node &start, const View &trg,
                 const SteerIterations &trials) const {
  auto traj = getTrajectory(start.state(), trg);
  if (nullptr == traj) {
    return std::nullopt;
  }
  std::vector<float> steered;
  float cost;
  bool trg_reached = false;
  for (std::size_t t = 0; t < trials.get(); ++t) {
    auto info = traj->advance();
    if (Trajectory::AdvanceInfo::blocked == info) {
      break;
    }
    steered = traj->getState().convert();
    cost = traj->getCumulatedCost();
    if (Trajectory::AdvanceInfo::targetReached == info) {
      trg_reached = true;
      break;
    }
  }
  if (steered.empty()) {
    return std::nullopt;
  }
  SteerResult res{NodeOwning{std::move(steered)}, trg_reached};
  res.node.setParent(start, cost);
  return res;
}
} // namespace mt_rrt