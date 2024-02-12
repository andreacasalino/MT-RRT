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

namespace mt_rrt {
struct KeepSearchPredicate {
  bool best_effort;
  std::size_t max_iterations;
  ExpansionStrategy strategy;
  std::atomic_bool one_solution_was_found = false;

  bool operator()(std::size_t iter) const;
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

/** @brief Used to extend one or two connected search trees
 */
class Extender : public ProblemAware {
public:
  virtual ~Extender() = default;

  /** @brief Perform the specified number of estensions on the wrapped tree(s).
   * This function may be called multiple times, for performing batch of
   * extensions. All the solutions found while extending are saved and stored in
   * this object.
   * @param the number of extension to perform
   */
  std::size_t search();

  virtual std::vector<TreeHandlerPtr> dumpTrees() = 0;

  const Solutions &getSolutions() const { return solutions; };
  Solutions &getSolutions() { return solutions; };

protected:
  Extender(const TreeHandler &handler);

  virtual void search_iteration() = 0;

  const Parameters &parameters;
  Solutions solutions;
  std::optional<DeterminismRegulator> determinism_manager;
};

using ExtenderPtr = std::unique_ptr<Extender>;
} // namespace mt_rrt
