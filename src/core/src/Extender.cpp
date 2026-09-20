/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Extender.h>

#ifdef SHOW_PLANNER_PROGRESS
#include "Progress.h"
#endif

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
    : deterministic_rate_sampler(0, 1.f, seed),
      deterministic_rate_sampler_threshold(determinism.get()) {}

Extender::Extender(const TreeHandler &handler)
    : ProblemAware(handler), parameters(handler.parameters) {}

std::size_t Extender::search() {
  determinism_manager.emplace(problem().sampler->sampleSeed(),
                              parameters.determinism);
  KeepSearchPredicate search_predicate =
      KeepSearchPredicate{parameters.best_effort, parameters.iterations.get(),
                          parameters.expansion_strategy};

  std::size_t iter = 0;
  for (; search_predicate(iter); ++iter) {
    search_iteration();
    search_predicate.one_solution_was_found = !solutions.empty();
#ifdef SHOW_PLANNER_PROGRESS
    ++Progress::get();
#endif
  }

  return iter;
}
} // namespace mt_rrt
