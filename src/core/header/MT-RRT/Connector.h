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

namespace mt_rrt {

/**
 * @brief Factory of optimal trajectories, refer to Section 1.3.0.1 of the
 * documentation.
 */
class Connector : public Copiable<Connector> {
public:
  Connector(const Connector &) = delete;
  Connector &operator=(const Connector &) = delete;

  /**
   * @param the starting node in the trajectory whose cost is to evaluate
   * @param the ending node in the trajectory whose cost is to evaluate
   * @return the cost C(\tau), Section 1.2 of the documentation, to traverse the
   * optimal trajectory connecting the passed pair of states. This cost does not
   * acoount for constraints, as this is done by minCost2GoConstrained(...)
   */
  virtual float minCost2Go(const View &start, const View &end) const = 0;

  /**
   * @param the starting node in the trajectory whose cost is to evaluate
   * @param the ending node in the trajectory whose cost is to evaluate
   * @return the cost C(\tau), Section 1.2 of the documentation, of
   * the trajectory \tau going from the starting node to the ending one, but
   * accounting for the constraints, Section 1.2 of the documentation.
   * COST_MAX is returned in this case returned when a feasible trajectory
   * exists, but is not entirely contained in the admitted set, Section 1.2 of
   * the documentation.
   */
  float minCost2GoConstrained(const View &start, const View &end) const;

  /**
   * @param the starting state
   * @param the ending state
   * @return the optimal trajectory connecting the passed states. nullptr is
   * returned in case a feasible trajectory does not exist
   */
  virtual TrajectoryPtr getTrajectory(const View &start,
                                      const View &end) const = 0;

  /**
   * @brief Performs a steering operation, Sections 1.2.1, 1.2.2 and 1.2.3 of
   * the documentation, from a staring node to a target one.
   * @param starting configuration
   * @param target configuration to reach
   * @return the steered configuration and a flag specifying whether target was
   * reached. Is a nullopt when the steering was not possible at all
   */
  struct SteerResult {
    bool target_is_reached;
    NodeOwning node;
  };
  std::optional<SteerResult> steer(const Node &start, const View &target,
                                   const SteerIterations &trials) const;

protected:
  Connector() = default;
};

using ConnectorPtr = std::unique_ptr<Connector>;
} // namespace mt_rrt
