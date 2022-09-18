/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-carpet/Copiable.h>
#include <MT-RRT-carpet/Limited.h>
#include <MT-RRT-core/Node.h>

#include <optional>

namespace mt_rrt {
using Cost = Positive<float>;
static constexpr float COST_MAX = std::numeric_limits<float>::max();

/**
 * @brief explanation:
 *  - blocked -> when the advancement is not anymore possible, i.e. last
 *    state reached is not admitted by constraints
 *  - advanced -> normal advancement. The state reached is admitted by
 *    constraints.
 *  - targetReached -> similar to advanced, but in case in the reached state is
 *    the ending one
 */
enum class AdvanceInfo { blocked, advanced, targetReached };

/**
 * @brief Interface describing an optimal trajectory \tau connecting 2 states,
 * Section 1.2 of the documentation, in a particular problem to solve.
 * After construction, the state on the trajectory is assumed at the beginning
 * of the trajectory itself. Calling advance(), shift the cursor along the
 * trajectory.
 */
class Trajectory {
public:
  virtual ~Trajectory() = default;

  Trajectory(const Trajectory &) = delete;
  Trajectory &operator=(const Trajectory &) = delete;

  /**
   * @brief Advance along the trajectory
   */
  virtual AdvanceInfo advance() = 0;

  /**
   * @return the current state on the trajectory.
   * IMPORTANT: it is a no-sense value in case last this->advance() returned
   * blocked
   */
  virtual State getState() const = 0;

  /**
   * @return the cost to go from the beginning of the trajectory to the current
   * reached state. IMPORTANT: it is a no-sense value in case last advance()
   * returned blocked
   */
  virtual float getCumulatedCost() const = 0;

protected:
  Trajectory() = default;
};

using TrajectoryPtr = std::unique_ptr<Trajectory>;

/**
 * @brief NUmber of times to try steer, refer to documentation at Section
 * "Background on RRT"
 */
class SteerIterations : public LowerLimited<std::size_t> {
public:
  SteerIterations();

  SteerIterations(const std::size_t trials);
};

struct SteerResult {
  NodePtr steered_node;
  bool target_is_reached;
};

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
  virtual float minCost2Go(const State &start, const State &end) const = 0;

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
  float minCost2GoConstrained(const State &start, const State &end) const;

  /**
   * @param the starting state
   * @param the ending state
   * @return the optimal trajectory connecting the passed states. nullptr is
   * returned in case a feasible trajectory does not exist
   */
  virtual TrajectoryPtr getTrajectory(const State &start,
                                      const State &end) const = 0;

  /**
   * @brief Performs a steering operation, Sections 1.2.1, 1.2.2 and 1.2.3 of
   * the documentation, from a staring node to a target one.
   * @param starting configuration
   * @param target configuration to reach
   * @return the steered configuration. Is a nullopt when the steering was not
   * possible at all
   */
  std::optional<SteerResult> steer(Node &start, const State &target,
                                   const SteerIterations &trials) const;

protected:
  Connector() = default;
};

using ConnectorPtr = std::unique_ptr<Connector>;
} // namespace mt_rrt
