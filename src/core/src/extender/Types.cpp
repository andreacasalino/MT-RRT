/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/extender/Types.h>

#ifdef SHOW_PLANNER_PROGRESS
#include "Progress.h"
#endif

namespace mt_rrt::extender {
DeterminismRegulator::DeterminismRegulator(Seed seed,
                                           const Determinism &determinism)
    : deterministic_rate_sampler(0, 1.f, seed),
      deterministic_rate_sampler_threshold(determinism.get()) {}
} // namespace mt_rrt
