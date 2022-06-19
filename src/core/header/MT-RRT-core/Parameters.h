/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/PlanningProblem.h>

namespace mt_rrt {
class Determinism : public Limited<float> {
public:
  Determinism(const float val) : Limited<float>(0, 1.0, val){};
};

class Iterations : public LowerLimited<std::size_t> {
public:
  Iterations() : LowerLimited<std::size_t>(1, 1000){};

  Iterations(const std::size_t iters) : LowerLimited<std::size_t>(1, iters){};
};

/** @brief The kind of rrt strategy to use, refer to the ones described in
 * Sections 1.2.1, 1.2.2 and 1.2.3 of the documentation
 */
enum class ExpansionStrategy { Single, Bidir, Star };

struct Parameters {
  ExpansionStrategy expansion_strategy;
  SteerIterations steer_trials;
  Iterations iterations;
  Determinism determinism = Determinism{0.35f};
  bool best_effort = true;
};

struct DescriptionAndParameters {
  const ProblemDescription &description;
  const Parameters &parameters;
};
} // namespace mt_rrt
