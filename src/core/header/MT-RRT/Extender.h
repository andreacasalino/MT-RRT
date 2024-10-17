/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Solution.h>
#include <MT-RRT/TreeHandler.h>

#include <atomic>
#include <optional>
#include <variant>

namespace mt_rrt {
struct KeepSearchPredicate {
  bool best_effort;
  std::size_t max_iterations;
  ExpansionStrategy strategy;
  std::atomic_bool one_solution_was_found = false;

  bool operator()(std::size_t iter) const {
    if ((strategy != ExpansionStrategy::Star) && best_effort &&
        one_solution_was_found) {
      return false;
    }
    return iter < max_iterations;
  }
};

class DeterminismRegulator {
public:
  DeterminismRegulator(const Seed &seed, const Determinism &determinism);

  bool doDeterministicExtension() const {
    return deterministic_rate_sampler.sample() <=
           deterministic_rate_sampler_threshold;
  }

private:
  UniformEngine deterministic_rate_sampler;
  const float deterministic_rate_sampler_threshold;
};

class ExtenderBase : public ProblemAware {
protected:
  ExtenderBase(const TreeHandler &handler);

  const Parameters &parameters;
  std::optional<DeterminismRegulator> determinism_manager;
};

class ExtenderSingle;
class ExtenderBidir;

template <typename Impl> struct ExtenderT : public Impl {
public:
  using Impl::Impl;

  /** @brief Perform the specified number of estensions on the wrapped tree(s).
   * This function may be called multiple times, for performing batch of
   * extensions. All the solutions found while extending are saved and stored in
   * this object.
   * @param the number of extension to perform
   */
  std::size_t search();

  Solutions<typename Impl::SolutionT> solutions;
};

using Extender =
    std::variant<ExtenderT<ExtenderSingle>, ExtenderT<ExtenderBidir>>;

std::vector<TreeHandlerPtr> dumpTrees(Extender &extender);

} // namespace mt_rrt
