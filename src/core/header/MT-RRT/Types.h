/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>
#include <MT-RRT/Limited.h>
#include <MT-RRT/Node.h>
#include <MT-RRT/ProblemDescription.h>
#include <MT-RRT/Random.h>
#include <MT-RRT/Types.h>

#include <algorithm>
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
 * @brief Groups together all the parameters that a @Planner neeeds to
 * know to solve a specific problem for connecting 2 pair of states.
 */
template <IsProblemDescription P> struct Problem : P {
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

struct KeepSearchPredicate {
  bool best_effort;
  std::size_t max_iterations;
  ExpansionStrategy strategy;
  std::atomic_bool one_solution_was_found = false;

  bool keepSearch(std::size_t iter) const;
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
  template <typename T, typename U>
  std::size_t
  operator()(const std::pair<const Node *, const float *> &p) const noexcept {
    std::size_t h1 = std::hash<T *>{}(p.first);
    std::size_t h2 = std::hash<U *>{}(p.second);

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

struct NearestNeighbour {
  std::span<const float> state_to_reach;
  const Node *closest = nullptr;
  float closestCost = COST_MAX;

  template <Connector C> void update(const Node &candidate, C &connector) {
    auto cost2Go =
        connector.makeTrajectory(candidate.data().state, state_to_reach)
            .minCost2Go()
            .get();
    if (cost2Go < closestCost) {
      closest = &candidate;
      closestCost = cost2Go;
    }
  }
};

struct NearSetElement {
  bool isRoot;
  const Node *element;
  Positive cost2Root;
  Positive cost2go;

  float costTot() const { return cost2Root.get() + cost2go.get(); }
};

struct NearSetHandler {
  NearSetHandler(Positive r, Node &pvt, std::vector<NearSetElement> &set)
      : ray{r.get()}, pivot{pvt}, near_set{set} {
    near_set.clear();
  }

  template <Connector C> void tryAdd(Node &node, C &connector) {
    auto traj =
        connector.makeTrajectory(candidate.data().state, state_to_reach);
    if (traj.has_value() && traj->minCost2Go().get() <= ray) {
      auto traversed = traj->traverse();
      if (traversed.has_value()) {
        float cost2Go = traversed->cost2Go.get();
        set.emplace_back(NearSetElement{node.data().parent == nullptr, &node,
                                        node.cost2Root(), cost2Go});
      }
    }
  }

  Positive ray;
  Node &pivot;
  std::vector<NearSetElement> &near_set;
};

struct Rewire {
  Node *involved_node;
  Positive updatedCost2Go;
};
} // namespace mt_rrt
