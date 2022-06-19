/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-carpet/Limited.h>

namespace mt_rrt {
class SynchronizationDegree : public Limited<float> {
public:
  SynchronizationDegree(const float initial_value);

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
