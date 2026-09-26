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
concept Connector = requires(C obj, std::span<const float> start,
                             std::span<const float> target) {
  typename C::trajectory_type;

  /**
   * @param the starting state
   * @param the ending state
   * @return the optimal trajectory connecting the passed states. nullptr is
   * returned in case a feasible trajectory does not exist
   */
  {
    obj.makeTrajectory(start, target)
    } -> std::same_as<std::optional<typename C::trajectory_type>>;
}
&&std::is_base_of_v<Copiable<C>, C>;
} // namespace mt_rrt
