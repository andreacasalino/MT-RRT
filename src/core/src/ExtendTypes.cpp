/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/ExtendTypes.h>

namespace mt_rrt {
bool KeepSearchPredicate::operator()(std::size_t iter) const {
  if ((strategy != ExpansionStrategy::Star) && best_effort &&
      one_solution_was_found) {
    return false;
  }
  return iter < max_iterations;
}

DeterminismRegulator::DeterminismRegulator(const Seed &seed,
                                           const Determinism &determinism)
    : deterministic_rate_sampler{0, 1.f, seed},
      deterministic_rate_sampler_threshold{determinism.get()} {}
} // namespace mt_rrt
