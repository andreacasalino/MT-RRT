/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Random.h>
#include <MT-RRT/Types.h>

#include <atomic>

namespace mt_rrt::extender {
struct KeepSearchPredicate {
  bool best_effort;
  std::size_t max_iterations;
  ExpansionStrategy strategy;
  std::atomic_bool one_solution_was_found = false;

  bool operator()(std::size_t iter) const {
    if ((strategy != ExpansionStrategy::Star) && best_effort &&
        one_solution_was_found.load(std::memory_order::memory_order_acquire)) {
      return false;
    }
    return iter < max_iterations;
  }
};

class DeterminismRegulator {
public:
  DeterminismRegulator(Seed seed, const Determinism &determinism);

  bool doDeterministicExtension() const {
    return deterministic_rate_sampler.sample() <=
           deterministic_rate_sampler_threshold;
  }

private:
  UniformEngine deterministic_rate_sampler;
  const float deterministic_rate_sampler_threshold;
};
} // namespace mt_rrt
