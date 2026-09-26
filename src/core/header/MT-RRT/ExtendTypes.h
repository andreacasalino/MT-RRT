/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>
#include <MT-RRT/Node.h>
#include <MT-RRT/Random.h>
#include <MT-RRT/Types.h>

#include <algorithm>
#include <atomic>
#include <unordered_set>

namespace mt_rrt {
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
};

struct NearSetHandler {
  static float nearSetRay(std::size_t tree_size, std::size_t problem_size,
                          const Positive &gamma) {
    const float tree_size_float = static_cast<float>(tree_size);
    return gamma.get() * powf(logf(tree_size_float) / tree_size_float,
                              1.f / static_cast<float>(problem_size));
  }

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

  float ray;
  Node &pivot;
  std::vector<NearSetElement> &near_set;
};

struct Rewire {
  Node *involved_node;
  Positive updatedCost2Go;
};
} // namespace mt_rrt
