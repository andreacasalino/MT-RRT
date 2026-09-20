/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>
#include <MT-RRT/Node.h>

#include <algorithm>
#include <atomic>
#include <unordered_set>

namespace mt_rrt {
struct NearSetElement {
  bool isRoot;
  Node *element;
  Positive cost2Root;
  Positive cost2go;
};

struct NearSet {
  Positive cost2RootSubject;
  std::vector<NearSetElement> set;
};

struct Rewire {
  Node *involved_node;
  Positive new_cost_from_father;
};

struct TreeData {
  Nodes<Node> nodes;

  // contains the register of nodes that were already deterministically
  // steered over a certain state
  //
  // keys are the steered node, while the values are the states
  // toward which the node were deterministically steered
  using DeterministicSteerRegister =
      std::unordered_set<std::pair<const Node *, const float *>>;
  DeterministicSteerRegister deterministic_steer_register;
};

struct NearestQuery {
  const Node *closest = nullptr;
  float closestCost = COST_MAX;

  void operator()(const Node &candidate, float cost2Go) {
    if (cost2Go < closestCost) {
      closest = &candidate;
      closestCost = cost2Go;
    }
  }
};

template <Connector C> struct NearSetQuery {
  float ray;
  std::span<const float> state_pivot;
  const C &connector;
  NearSet &recipient;

  void operator()(Node &subject) {
    if (connector->minCost2Go(subject.state(), state_pivot) <= ray) {
      float cost2Go =
          connector->minCost2GoConstrained(subject.state(), state_pivot);
      if (cost2Go == COST_MAX)
        return;
      set.emplace_back(NearSetElement{subject.getParent() == nullptr, &subject,
                                      subject.cost2Root(), cost2Go});
    }
  }
};

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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float near_set_ray(std::size_t tree_size, std::size_t problem_size,
                   float gamma);

template <Connector C>
const Node *
nearest_neighbour(std::span<const float> state,
                  typename std::deque<Node>::const_iterator nodes_begin,
                  typename std::deque<Node>::const_iterator nodes_end,
                  const C &connector) {
  NearestQuery query;
  std::for_each(nodes_begin, nodes_end, [&](const auto &node) {
    query(node, connector.minCost2Go(node.data().state, state).get());
  });
  return query.closest;
}

template <Connector C>
void near_set(std::span<const float> state,
              typename std::deque<Node>::iterator nodes_begin,
              typename std::deque<Node>::iterator nodes_end, std::size_t size,
              C &connector, NearSet &recipient, const Positive &gamma) {
  recipient.set.clear();
  float ray = near_set_ray(size, state.size(), gamma.get());
  NearSetQuery res{ray, state, connector, recipient};
  std::for_each(nodes_begin, nodes_end, [](auto &node) { res(node); });
}

std::vector<Rewire> compute_rewires(Node &candidate, NearSet &&near_set,
                                    const DescriptionAndParameters &context);

std::optional<Connector::SteerResult> extend(const View &target,
                                             TreeHandler &tree_handler,
                                             const bool is_deterministic);

std::optional<Connector::SteerResult> extend_star(const View &target,
                                                  TreeHandler &tree_handler,
                                                  const bool is_deterministic,
                                                  std::vector<Rewire> &rewires);

void apply_rewires_if_better(const Node &parent,
                             const std::vector<Rewire> &rewires);
} // namespace mt_rrt
