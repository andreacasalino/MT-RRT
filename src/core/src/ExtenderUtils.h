/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/Node.h>
#include <MT-RRT-core/Planner.h>

#include <atomic>
#include <unordered_map>

namespace mt_rrt {
struct KeepSearchPredicate {
  bool best_effort;
  std::size_t max_iterations;
  ExpansionStrategy strategy;
  std::atomic_bool one_solution_was_found = false;

  bool operator()(const std::size_t iter) const;
};

Node *nearest_neighbour(const State &state,
                        const Tree::const_reverse_iterator &tree_begin,
                        const Tree::const_reverse_iterator &tree_end,
                        const DescriptionAndParameters &context);

float near_set_ray(const std::size_t tree_size, const std::size_t problem_size,
                   float gamma);

using NearSet =
    std::unordered_map<Node *, float>; // node, cost to root at the time the
                                       // near site was computed
NearSet near_set(const Node &subject,
                 const Tree::const_reverse_iterator &tree_begin,
                 const Tree::const_reverse_iterator &tree_end,
                 const DescriptionAndParameters &context);

struct Rewire {
  Node &involved_node;
  float new_cost_from_father;
};
using Rewires = std::vector<Rewire>;
Rewires compute_rewires(Node &just_steered, const NearSet &near_set,
                        const DescriptionAndParameters &context);
} // namespace mt_rrt
