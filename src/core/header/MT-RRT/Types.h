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
 * @brief used to regulate the deterministic bias, see documentation at Section
 * "Background on RRT"
 */
class Determinism : public Limited<float> {
public:
  Determinism(const float val) : Limited<float>(0, 1.0, val){};
};

/**
 * @brief iterations limits to find a solution.
 */
class Iterations : public LowerLimited<std::size_t> {
public:
  Iterations() : LowerLimited<std::size_t>(1, 1000){};

  Iterations(const std::size_t iters) : LowerLimited<std::size_t>(1, iters){};
};

/**
 * @brief The kind of strategy to use, refer to documentation at
 * Sections 1.2.1, 1.2.2 and 1.2.3
 */
enum class ExpansionStrategy { Single, Bidir, Star };

using Cost = Positive<float>;
static constexpr float COST_MAX = std::numeric_limits<float>::max();

/**
 * @brief NUmber of times to try steer, refer to documentation at Section
 * "Background on RRT"
 */
class SteerIterations : public LowerLimited<std::size_t> {
public:
  SteerIterations();

  SteerIterations(const std::size_t trials);
};

/**
 * @brief Groups together all the parameters that a @Planner neeeds to
 * know to solve a specific problem for connecting 2 pair of states.
 */
struct Parameters {
  ExpansionStrategy expansion_strategy = ExpansionStrategy::Star;
  SteerIterations steer_trials;
  Iterations iterations;
  Determinism determinism = Determinism{0.35f};
  /**
   * @brief If true, the expansion of the tree(s) is arrested as soon as a
   * solution is found. Otherwise, the search is kept on possibly finding
   * additional solutions.
   */
  bool best_effort = true;
};
} // namespace mt_rrt
