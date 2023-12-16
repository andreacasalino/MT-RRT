/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Limited.h>

namespace mt_rrt {
/**
 * @brief Synchronization degree used by MultiAgentPlanner and
 * LinkedTreesPlanner. This regulates how often the threads stop expansions and
 * share info about explored states with other threads.
 */
class SynchronizationDegree : public Limited<float> {
public:
  SynchronizationDegree(float initial_value);

  SynchronizationDegree();
};

class SynchronizationAware {
public:
  virtual ~SynchronizationAware() = default;

  SynchronizationDegree &synchronization() { return synchronization_; };
  const SynchronizationDegree &synchronization() const {
    return synchronization_;
  };

protected:
  SynchronizationAware() = default;

private:
  SynchronizationDegree synchronization_ = SynchronizationDegree{};
};
} // namespace mt_rrt
