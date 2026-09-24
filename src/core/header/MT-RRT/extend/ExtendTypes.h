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

  bool doDeterministicExtension() const {
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
    Positive new_cost_from_father;
  };

  Rewiring(float gamma, std::size_t state_space_size);

  const auto &get() const { return data_; }

  template <Connector C, typename NodesIter>
  void update(std::span<const float> state, NodesIter nodes_begin,
              NodesIter nodes_end, std::size_t tree_size, const C &connector) {
    float ray = computeRay(tree_size);

    data_.state_pivot = state;
    data_.set.clear();

    std::for_each(nodes_begin, nodes_end, [&](const Node &subject) {
      if (connector.minCost2Go(subject.state(), state_pivot) <= ray) {
        float cost2Go =
            connector.minCost2GoConstrained(subject.state(), state_pivot);
        if (cost2Go == COST_MAX)
          return;
        set.emplace_back(NearSetElement{subject.getParent() == nullptr,
                                        &subject, subject.cost2Root(),
                                        cost2Go});
      }
    });
  }

  void compute_rewires(Node &candidate, NearSet &&near_set,
                       const DescriptionAndParameters &context);

  // For each rewire cancidate, it applies it only if that is actually beffer
  // than current connections
  void apply_rewires(const Node &parent, const std::vector<Rewire> &rewires);

  // re-usable buffers
  Node *pivot{nullptr};
  std::vector<NearSetElement> near_set_;
  std::vector<Rewire> rewires_;

private:
  float gamma_;
  std::size_t state_space_size_;
};
} // namespace mt_rrt
