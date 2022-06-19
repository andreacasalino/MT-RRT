/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <ExtenderUtils.h>
#include <MT-RRT-carpet/Error.h>

#include <algorithm>
#include <map>
#include <math.h>

namespace mt_rrt {
bool KeepSearchPredicate::operator()(const std::size_t iter) const {
  if ((strategy != ExpansionStrategy::Star) && best_effort &&
      one_solution_was_found) {
    return false;
  }
  return iter < max_iterations;
}

Node *nearest_neighbour(const State &state,
                        const Tree::const_reverse_iterator &tree_begin,
                        const Tree::const_reverse_iterator &tree_end,
                        const DescriptionAndParameters &context) {
  const auto &connector = *context.description.connector;
  Node *result = nullptr;
  float nearest_cost = COST_MAX;
  std::for_each(tree_begin, tree_end, [&](const NodePtr &node) {
    float cost = connector.minCost2Go(node->getState(), state);
    if (cost < nearest_cost) {
      result = node.get();
      nearest_cost = cost;
    }
  });
  return result;
}

float near_set_ray(const std::size_t tree_size, const std::size_t problem_size,
                   float gamma) {
  const float tree_size_float = static_cast<float>(tree_size);
  return gamma * powf(logf(tree_size_float) / tree_size_float,
                      1.f / static_cast<float>(problem_size));
}

NearSet near_set(const Node &subject,
                 const Tree::const_reverse_iterator &tree_begin,
                 const Tree::const_reverse_iterator &tree_end,
                 const DescriptionAndParameters &context) {
  const auto &connector = *context.description.connector;
  float ray = near_set_ray(std::distance(tree_begin, tree_end),
                           (*tree_begin)->getState().size(),
                           context.description.gamma.get());
  NearSet result;
  const auto &state = subject.getState();
  std::for_each(tree_begin, tree_end, [&](const NodePtr &node) {
    if (connector.minCost2Go(node->getState(), state) <= ray) {
      result.emplace(node.get(), node->cost2Root());
    }
  });
  auto *subject_father = subject.getFatherInfo().father;
  if (result.find(subject_father) == result.end()) {
    result.emplace(subject_father, subject_father->cost2Root());
  }
  return result;
}

Rewires compute_rewires(Node &just_steered, const NearSet &near_set,
                        const DescriptionAndParameters &context) {
  const auto near_set_just_steered_father_it =
      near_set.find(just_steered.getFatherInfo().father);
  if (near_set_just_steered_father_it == near_set.end()) {
    throw Error{"Info about the father of the just steered node were not part "
                "of the near set"};
  }
  if (near_set.size() == 1) {
    return {};
  }

  const auto &connector = *context.description.connector;

  // sort elements in near set according to cost to root
  std::multimap<float, NodeFatherInfo> just_steered_cost_to_root_map;
  float just_steered_cost_to_root =
      near_set_just_steered_father_it->second +
      just_steered.getFatherInfo().cost_from_father;
  {
    for (const auto &[node, node_cost_to_root] : near_set) {
      if (node == just_steered.getFatherInfo().father) {
        // in order to avoid recomputation of cost to go constrained
        continue;
      }
      const float cost_to_go_constrained = connector.minCost2GoConstrained(
          node->getState(), just_steered.getState());
      if (cost_to_go_constrained == COST_MAX) {
        continue;
      }
      just_steered_cost_to_root_map.emplace(
          cost_to_go_constrained + node_cost_to_root,
          NodeFatherInfo{node, cost_to_go_constrained});
    } // namespace mt_rrt
    just_steered_cost_to_root_map.emplace(just_steered_cost_to_root,
                                          just_steered.getFatherInfo());
  }

  // if best in near set is not current father, rewire just_steered
  if (just_steered_cost_to_root_map.begin()->second.father !=
      just_steered.getFatherInfo().father) {
    const auto cost_map_begin = just_steered_cost_to_root_map.begin();
    auto *new_father = cost_map_begin->second.father;
    just_steered.setFatherInfo(
        NodeFatherInfo{new_father, cost_map_begin->second.cost_from_father});
    just_steered_cost_to_root = near_set.find(new_father)->second +
                                cost_map_begin->second.cost_from_father;
  }
  just_steered_cost_to_root_map.erase(just_steered_cost_to_root_map.begin());

  // check for rewires
  const bool symmetric = context.description.simmetry;
  Rewires result;
  for (const auto &[c, node_info] : just_steered_cost_to_root_map) {
    auto *node = node_info.father;
    if (nullptr == node->getFatherInfo().father) {
      // root can't be rewired
      continue;
    }
    const float cost_to_go_constrained =
        symmetric ? node_info.cost_from_father
                  : connector.minCost2GoConstrained(just_steered.getState(),
                                                    node->getState());
    if (cost_to_go_constrained == COST_MAX) {
      continue;
    }
    const float by_pass_cost2Root =
        just_steered_cost_to_root + cost_to_go_constrained;
    if (by_pass_cost2Root < near_set.find(node)->second) {
      result.emplace_back(Rewire{*node, cost_to_go_constrained});
    }
  }
  return result;
}
} // namespace mt_rrt
