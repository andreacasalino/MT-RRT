/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <memory>
#include <span>

namespace mt_rrt {
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
   * @brief explanation:
   *  - blocked -> when the advancement is not anymore possible, i.e. last
   *    state reached is not admitted by constraints
   *  - advanced -> normal advancement. The state reached is admitted by
   *    constraints.
   *  - targetReached -> similar to advanced, but in case in the reached state
   * is the ending one
   */
  enum class AdvanceInfo { Blocked, Advanced, TargetReached };

  /**
   * @brief Advance along the trajectory
   */
  virtual AdvanceInfo advance() = 0;

  /**
   * @return the current state on the trajectory.
   * IMPORTANT: it is a no-sense value in case last this->advance() returned
   * blocked
   */
  virtual std::span<const float> getState() const = 0;

  /**
   * @return the cost to go from the beginning of the trajectory to the current
   * reached state. IMPORTANT: it is a no-sense value in case last advance()
   * returned blocked
   */
  virtual float getCumulatedCost() const = 0;
};

using TrajectoryPtr = std::unique_ptr<Trajectory>;
} // namespace mt_rrt
