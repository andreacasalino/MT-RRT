/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Node.h>
#include <MT-RRT/NodesIterator.hxx>
#include <MT-RRT/Random.h>
#include <MT-RRT/concepts/Connector.h>

#include <algorithm>
#include <atomic>
#include <unordered_set>

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

  bool shallThisBeDeterministic() const {
    return deterministic_rate_sampler.sample() <=
           deterministic_rate_sampler_threshold;
  }

private:
  UniformEngine deterministic_rate_sampler;
  const float deterministic_rate_sampler_threshold;
};

struct NearestNeighbour {
  const Node *closest = nullptr;
  float closestCost = COST_MAX;

  void update(const Node &candidate, float cost2Go) {
    if (cost2Go < closestCost) {
      closest = &candidate;
      closestCost = cost2Go;
    }
  }
};

struct Rewiring {
  struct NearSetElement {
    bool isRoot;
    const Node *element;
    Positive cost2Root;
    Positive cost2go;
  };

  struct Rewire {
    Node *involved_node;
    Positive updatedCost2Go;
  };

  Rewiring(float gamma, std::size_t state_space_size);

  void reset(std::span<const float> pivot, std::size_t tree_size);

  // TODO
  // - internally compute near set getting from outside the connector and tree,
  // then compute rewires without executing them
  // - once done, get the computed rewires and apply them (from outside)

  template <Connector C, typename T>
  void
  update(const T &tree,
         Connector &connector); // internally check it Tree has iter() or not

  // template <Connector C>
  // void updateNearSet(const Node &candidate, const C &connector) {
  //   if (connector.minCost2Go(candidate.state(), state_pivot) <= ray) {
  //     float cost2Go =
  //         connector.minCost2GoConstrained(candidate.state(), state_pivot);
  //     if (cost2Go == COST_MAX)
  //       return;
  //     set.emplace_back(NearSetElement{candidate.getParent() == nullptr,
  //                                     &candidate, candidate.cost2Root(),
  //                                     cost2Go});
  //   }
  // }

  // // // void compute_rewires(Node &candidate, NearSet &&near_set,
  // // //                      const DescriptionAndParameters &context);

  // // // // For each rewire cancidate, it applies it only if that is actually
  // beffer
  // // // // than current connections
  // // // void apply_rewires(const Node &parent, const std::vector<Rewire>
  // &rewires);

  // scratch buffers
  struct Data {
    std::span<const float> pivot;
    std::vector<NearSetElement> near_set;
    std::vector<Rewire> rewires;
  };

  const auto &get() const { return data_; }

private:
  float gamma_;
  std::size_t state_space_size_;

  Data data_;
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
// keys are the steered node, while the values are the states
// toward which the node were deterministically steered
using DeterministicSteerRegister =
    std::unordered_set<std::pair<const Node *, const float *>,
                       DeterministicSteerRegisterHash>;
} // namespace mt_rrt
