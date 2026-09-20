/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Copiable.h>
#include <MT-RRT/Node.h>
#include <MT-RRT/Trajectory.h>
#include <MT-RRT/Types.h>

#include <optional>
#include <span>
#include <vector>

namespace mt_rrt {
/**
 * @brief Factory of optimal trajectories, refer to Section 1.3.0.1 of the
 * documentation.
 */
template <typename C>
concept Connector = requires(const C obj_const, std::span<const float> start,
                             std::span<const float> target) {

  /**
   * @param the starting node in the trajectory whose cost is to evaluate
   * @param the ending node in the trajectory whose cost is to evaluate
   * @return the cost C(\tau), Section 1.2 of the documentation, to traverse the
   * optimal trajectory connecting the passed pair of states. This cost does not
   * acoount for constraints, as this is done by minCost2GoConstrained(...)
   */
  { obj_const.minCost2Go(start, target) } -> std::same_as<Positive>;

  /**
   * @param the starting state
   * @param the ending state
   * @return the optimal trajectory connecting the passed states. nullptr is
   * returned in case a feasible trajectory does not exist
   */
  {
    obj_const.getTrajectory(start, target)
    } -> std::same_as<std::optional<typename C::Trajectory>>;
}
&&std::is_base_of_v<Copiable<C>, C>;

/**
 * @param the connector
 * @param the starting node in the trajectory whose cost is to evaluate
 * @param the ending node in the trajectory whose cost is to evaluate
 * @return the cost C(\tau), Section 1.2 of the documentation, of
 * the trajectory \tau going from the starting node to the ending one, but
 * accounting for the constraints, Section 1.2 of the documentation.
 * COST_MAX is returned in this case returned when a feasible trajectory
 * exists, but is not entirely contained in the admitted set, Section 1.2 of
 * the documentation.
 */
template <Connector C>
Positive minCost2GoConstrained(const C &connector, std::span<const float> start,
                               std::span<const float> target) {
  Positive distance = connector.minCost2Go(start, target);
  if (COST_MAX == distance.get()) {
    return distance;
  }

  auto traj = connector.getTrajectory(start, target);
  if (!traj.has_value()) {
    throw Error{"found null trajectory but the min cost 2 go was not infinite"};
  }
  AdvanceInfo advInfo = traj.advance();
  for (; std::holds_alternative<Advanced>(advInfo); advInfo = traj->advance()) {
  }
  return std::holds_alternative<BlockedTag>(advInfo) ? COST_MAX : distance;
}

struct SteerResult {
  bool target_reached{false};
  Positive cost2Go{0};
};

/**
 * @return nullopt when no steer was actually possible
 */
template <Connector C>
[[nodiscard]] std::optional<SteerResult>
steer(const C &connector, std::vector<float> &reached_state,
      std::span<const float> start, std::span<const float> target,
      const SteerIterations &trials) {
  reached_state.clear();
  auto traj = connector.getTrajectory(start, target);
  if (!traj.has_value()) [[unlikely]] {
    return std::nullopt;
  }

  std::optional<SteerResult> res;
  auto set_result_ = [&](std::span<const float> state, const Positive &c) {
    reached_state.clear();
    reached_state.insert(reached_state.end(), state.begin(), state.end());
    res.emplace().cost2Go = c;
  };

  bool steered{false};
  for (std::size_t t = 0; t < trials.get(); ++t) {
    bool stop = std::visit(
        [&](auto &&advancement) {
          if constexpr (std::is_same_v<decltype(advancement), BlockedTag &&>) {
            return true;
          }

          else if constexpr (std::is_same_v<decltype(advancement),
                                            Advanced &&>) {
            set_result_(advancement.reached_state, advancement.cumulatedCost);
            return false;
          }

          else {
            set_result_(target, advancement.cumulatedCost);
            res->target_reached = true;
            return true;
          }
        },
        traj->advance());
    if (stop) {
      break;
    }
  }
  return res;
}
} // namespace mt_rrt
