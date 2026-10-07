/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Limited.h>
#include <MT-RRT/Node.h>
#include <MT-RRT/Random.h>
#include <MT-RRT/Types.h>

#include <atomic>
#include <unordered_set>

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
 * @brief The kind of strategy to use, refer to documentation at
 * Sections 1.2.1, 1.2.2 and 1.2.3
 */
enum class ExpansionStrategy { Single, Bidir, Star };

template <ExpansionStrategy ExpansionStrategyT> struct KeepSearchPredicate {
  const ProblemParameters &parameters;

  std::atomic_bool one_solution_was_found = false;

  [[nodiscard]] bool keepSearch(std::size_t iter) const;
};

class DeterminismRegulator {
public:
  DeterminismRegulator(const Seed &seed, const Determinism &determinism);

  bool shallThisBeDeterministic() const {
    return deterministic_rate_sampler.sample() <=
           deterministic_rate_sampler_threshold;
  }

private:
  UniformEngine deterministic_rate_sampler;
  const float deterministic_rate_sampler_threshold;
};

struct DeterministicSteerRegisterHash {
  std::size_t
  operator()(const std::pair<const Node *, const float *> &p) const noexcept {
    static std::hash<const Node *> node_ptr_hasher;
    static std::hash<const float *> state_ptr_hasher;
    std::size_t h1 = node_ptr_hasher(p.first);
    std::size_t h2 = state_ptr_hasher(p.second);

    // Combine the hashes using a high-quality mixing formula (from Boost)
    // This avoids collisions like (A, B) having the same hash as (B, A) if T ==
    // U
    return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
  }
};

// contains the register of nodes that were already deterministically
// steered over a certain state
//
// keys are the pair steered node - the states toward which the node were
// deterministically steered
using DeterministicSteerRegister =
    std::unordered_set<std::pair<const Node *, const float *>,
                       DeterministicSteerRegisterHash>;
} // namespace mt_rrt
