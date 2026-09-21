/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Node.h>
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

// struct TreeData {
//   Nodes<Node> nodes;

//   // contains the register of nodes that were already deterministically
//   // steered over a certain state
//   //
//   // keys are the steered node, while the values are the states
//   // toward which the node were deterministically steered
//   using DeterministicSteerRegister =
//       std::unordered_set<std::pair<const Node *, const float *>>;
//   DeterministicSteerRegister deterministic_steer_register;
// };

struct NearestNeighbour {
  const Node *closest = nullptr;
  float closestCost = COST_MAX;

  void update(const Node &candidate, float cost2Go) {
    if (cost2Go < closestCost) {
      closest = &candidate;
      closestCost = cost2Go;
    }
  }

  template <Connector C, typename NodesIter>
  NearestNeighbour perform(std::span<const float> state, NodesIter nodes_begin,
                           NodesIter nodes_end, const C &connector) {
    NearestQuery query;
    std::for_each(nodes_begin, nodes_end, [&](const auto &node) {
      query.update(node, connector.minCost2Go(node.data().state, state).get());
    });
    return query;
  }
};

struct NearSet {
  struct NearSetElement {
    bool isRoot;
    const Node *element;
    Positive cost2Root;
    Positive cost2go;
  };

  std::size_t problem_size;
  float gamma;

  float ray;
  std::span<const float> state_pivot;
  // Positive cost2RootSubject;
  std::vector<NearSetElement> set; // scratch buffer

  template <Connector C> void update(const Node &subject, const C &connector) {
    if (connector->minCost2Go(subject.state(), state_pivot) <= ray) {
      float cost2Go =
          connector->minCost2GoConstrained(subject.state(), state_pivot);
      if (cost2Go == COST_MAX)
        return;
      set.emplace_back(NearSetElement{subject.getParent() == nullptr, &subject,
                                      subject.cost2Root(), cost2Go});
    }
  }

  void clear(std::size_t tree_size) {
    ray = getRay(tree_size);
    state_pivot = {};
    set.clear();
  }

  template <Connector C, typename NodesIter>
  void compute(std::span<const float> state, NodesIter nodes_begin,
               NodesIter nodes_end, std::size_t tree_size, const C &connector) {
    clear(tree_size);
    std::for_each(nodes_begin, nodes_end,
                  [&](auto &node) { update(node, connector); });
  }

private:
  float getRay(std::size_t tree_size) const;
};

struct Rewire {
  Node *involved_node;
  Positive new_cost_from_father;
};
} // namespace mt_rrt
