/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Limited.h>
#include <span>
#include <variant>

namespace mt_rrt {
struct BlockedTag;
struct Advanced {
  std::span<float> reached_state;
  Positive cumulatedCost;
};
struct TargetReached {
  Positive cumulatedCost;
};
/**
 * @brief explanation:
 *  - blocked -> when the advancement is not anymore possible, i.e. last
 *    state reached is not admitted by constraints
 *  - advanced -> normal advancement. The state reached is admitted by
 *    constraints.
 *  - targetReached -> similar to advanced, but in case in the reached
 state
 * is the ending one
 */
using AdvanceInfo = std::variant<BlockedTag, Advanced, TargetReached>;

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
   * @brief Advance along the trajectory toward the target state
   */
  { obj.advance() } -> std::same_as<AdvanceInfo>;

  { obj_const.targetState() } -> std::same_as<std::span<const float>>;
};
} // namespace mt_rrt
