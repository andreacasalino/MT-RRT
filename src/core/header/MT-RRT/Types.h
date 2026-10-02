/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Limited.h>
#include <MT-RRT/ProblemDescription.h>

namespace mt_rrt {
/**
 * @brief used to regulate the deterministic bias, see documentation at Section
 * "Background on RRT"
 */
using Determinism = Limited<float, 0.f, 1.f>;

template <std::size_t Default>
struct PositiveIntegerWithDefault
    : Limited<std::size_t, 1, std::numeric_limits<std::size_t>::max()> {
  PositiveIntegerWithDefault() : PositiveIntegerWithDefault{Default} {}

  PositiveIntegerWithDefault(std::size_t value)
      : Limited<std::size_t, 1, std::numeric_limits<std::size_t>::max()>{
            value} {}
};

/**
 * @brief iterations limits to find a solution.
 */
using Iterations = PositiveIntegerWithDefault<1000>;

using Cost = Positive;
static constexpr float COST_MAX = std::numeric_limits<float>::max();

/**
 * @brief Number of times to try steer, refer to documentation at Section
 * "Background on RRT"
 */
using SteerIterations = PositiveIntegerWithDefault<10>;

/**
 * @brief Groups together all the parameters that a @Planner neeeds to
 * know to solve a specific problem for connecting 2 pair of states.
 */
template <Connector C, Sampler S, bool Simmetry,
          ExpansionStrategy ExpansionStrategyT>
struct Problem : ProblemDescription<C, S, Simmetry, ExpansionStrategyT> {
  Iterations iterations;
  Determinism determinism{0.35f};

  /**
   * @brief If true, the expansion of the tree(s) is arrested as soon as a
   * solution is found. Otherwise, the search is kept on possibly finding
   * additional solutions.
   */
  bool best_effort{true};

  bool provide_extra_info{false};
};
} // namespace mt_rrt
