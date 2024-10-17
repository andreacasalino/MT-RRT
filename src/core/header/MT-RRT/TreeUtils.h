/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/TreeHandler.h>

#include <algorithm>

namespace mt_rrt {
float near_set_ray(std::size_t tree_size, std::size_t problem_size,
                   float gamma);

struct NearestQuery {
  const Node *closest = nullptr;
  float closestCost = COST_MAX;

  void process(const Node &candidate, float cost2Go) {
    if (cost2Go < closestCost) {
      closest = &candidate;
      closestCost = cost2Go;
    }
  }
};

template <typename IterT>
const Node *nearest_neighbour(const View &state, IterT tree_begin,
                              IterT tree_end,
                              const DescriptionAndParameters &context) {
  const auto &connector = *context.description.connector;
  NearestQuery query;
  std::for_each(tree_begin, tree_end, [&](const Node *node) {
    query.process(*node, connector.minCost2Go(node->state(), state));
  });
  return query.closest;
}

struct NearSetQuery {
  float ray;
  View state_pivot;
  Connector *connector;
  std::vector<NearSetElement> set;

  template <bool ComputeCost2Root> void process(Node &subject) {
    if (connector->minCost2Go(subject.state(), state_pivot) <= ray) {
      float cost2Go =
          connector->minCost2GoConstrained(subject.state(), state_pivot);
      if (cost2Go == COST_MAX)
        return;
      float cost2Root;
      if constexpr (ComputeCost2Root) {
        cost2Root = subject.cost2Root();
      }
      set.emplace_back(NearSetElement{subject.getParent() == nullptr, &subject,
                                      cost2Root, cost2Go});
    }
  }
};

template <typename IterT>
std::vector<NearSetElement> near_set(const View &state, IterT tree_begin,
                                     IterT tree_end, std::size_t size,
                                     const DescriptionAndParameters &context) {
  std::size_t problem_size = (*tree_begin)->state().size;
  float ray = near_set_ray(size, problem_size, context.description.gamma.get());
  NearSetQuery res{ray, state, context.description.connector.get()};
  std::for_each(tree_begin, tree_end,
                [&res](Node *ptr) { res.template process<true>(*ptr); });
  return std::move(res.set);
}

Rewires compute_rewires(Node &candidate, NearSet &&near_set,
                        const DescriptionAndParameters &context);

std::optional<Connector::SteerResult> extend(const View &target,
                                             TreeHandler &tree_handler,
                                             const bool is_deterministic);

std::optional<Connector::SteerResult> extend_star(const View &target,
                                                  TreeHandler &tree_handler,
                                                  const bool is_deterministic,
                                                  Rewires &rewires);

void apply_rewires_if_better(const Node &parent, const Rewires &rewires);
} // namespace mt_rrt
