/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Limited.h>

#include <optional>
#include <span>

namespace mt_rrt {
struct TraverseResult {
  bool target_was_reached;

  /**
   * the last valid state along \tau, that is reachable accounting for
   * all constraints
   */
  std::span<float> reached_state;

  /**
   * the total cost to go from the starting state to the rached one
   */
  Positive cost2Go;
};

/**
 * @brief What constitutes an optimal trajectory \tau connecting 2 states,
 * Section 1.2 of the documentation, in a particular problem to solve.
 * After construction, the state on the trajectory is assumed at the beginning
 * of the trajectory itself. Calling advance(), shift the cursor along the
 * trajectory.
 */
template <typename T>
concept Trajectory = requires(T obj, const T obj_const) {
  /**
   * @return the cost C(\tau), Section 1.2 of the documentation, to traverse the
   * optimal trajectory connecting the passed pair of states. This cost does not
   * account for constraints, as this is done by minCost2GoConstrained(...).
   */
  { obj_const.minCost2Go() } -> std::same_as<Positive>;

  { obj.traverse() } -> std::same_as<std::optional<TraverseResult>>;
};
} // namespace mt_rrt
