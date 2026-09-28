/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Copiable.h>
#include <MT-RRT/Node.h>
#include <MT-RRT/Types.h>

#include <optional>
#include <span>
#include <vector>

namespace mt_rrt {
struct TraverseResult {
  bool target_was_reached;

  /**
   * the total cost to go from the starting state to the rached one
   */
  Positive cost2Go;
};

/**
 * @brief Factory of optimal trajectories, refer to Section 1.3.0.1 of the
 * documentation.
 */
template <typename C>
concept Connector = requires(C obj, std::span<const float> start,
                             std::span<const float> target,
                             std::vector<float> &reached) {
  /**
   * @return the cost C(\tau), Section 1.2 of the documentation, to traverse the
   * optimal trajectory connecting the passed pair of states. This cost does not
   * account for constraints, as this is done by minCost2GoConstrained(...).
   */
  { obj.minCost2Go() } -> std::same_as<Positive>;

  /**
   * @param the last valid state along \tau, i.e. the optimal trjectory
   * connecting start and target, accounting for all constraints
   *
   * @returns nullopt if no steer at all is possible
   */
  {
    obj.steer(start, target, reached)
    } -> std::same_as<std::optional<TraverseResult>>;
}
&&std::is_base_of_v<Copiable<C>, C>;
} // namespace mt_rrt
