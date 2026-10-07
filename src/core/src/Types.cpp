/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Types.h>

namespace mt_rrt {
template <ExpansionStrategy ExpansionStrategyT>
bool KeepSearchPredicate<ExpansionStrategyT>::keepSearch(
    std::size_t iter) const {

  if constexpr (ExpansionStrategyT != ExpansionStrategy::Star) {
    if (parameters.best_effort &&
        one_solution_was_found.load(std::memory_order_relaxed)) {
      return false;
    }
  }

  return iter < parameters.iterations.get();
}

DeterminismRegulator::DeterminismRegulator(const Seed &seed,
                                           const Determinism &determinism)
    : deterministic_rate_sampler{0, 1.f, seed},
      deterministic_rate_sampler_threshold{determinism.get()} {}
} // namespace mt_rrt
