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
#include <MT-RRT/concepts/Trajectory.h>

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
   * account for constraints, as this is done by minCost2GoConstrained(...)
   */
  { obj_const.minCost2Go(start, target) } -> std::same_as<Positive>;

  typename C::trajectory_type;

  /**
   * @param the starting state
   * @param the ending state
   * @return the optimal trajectory connecting the passed states. nullptr is
   * returned in case a feasible trajectory does not exist
   */
  {
    obj_const.getTrajectory(start, target)
    } -> std::same_as<std::optional<typename C::trajectory_type>>;
}
&&std::is_base_of_v<Copiable<C>, C>;
} // namespace mt_rrt
