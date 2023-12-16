/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/ExtenderUtils.h>

namespace mt_rrt {
float near_set_ray(std::size_t tree_size, std::size_t problem_size,
                   float gamma) {
  const float tree_size_float = static_cast<float>(tree_size);
  return gamma * powf(logf(tree_size_float) / tree_size_float,
                      1.f / static_cast<float>(problem_size));
}

std::vector<Rewire> compute_rewires(Node &subject, NearSet &&near_set_info,
                                    const DescriptionAndParameters &context) {
  auto &near_set = near_set_info.set;
  if (near_set.empty()) {
    return {};
  }

  const auto &connector = *context.description.connector;
  float cost2RootSubject = near_set_info.cost2RootSubject;

  // rewire just_steered to the best father
  if (auto it = std::min_element(
          near_set.begin(), near_set.end(),
          [](const NearSetElement &a, const NearSetElement &b) {
            return a.cost2go + a.cost2Root < b.cost2go + b.cost2Root;
          });
      it->cost2go + it->cost2Root < cost2RootSubject) {
    subject.setParent(*it->element, it->cost2go);
    cost2RootSubject = it->cost2go + it->cost2Root;
  }
  // remove current parent from rewire candidates
  if (auto it =
          std::find_if(near_set.begin(), near_set.end(),
                       [parent = subject.getParent()](const NearSetElement &e) {
                         return e.element == parent;
                       });
      it != near_set.end()) {
    near_set.erase(it);
  }

  // check for rewires
  bool symmetric = context.description.simmetry;
  std::vector<Rewire> res;
  for (auto [isRoot, node, nodeCost2Root, cost2GoPrev] : near_set) {
    if (isRoot) {
      // root can't be rewired
      continue;
    }
    float cost2Go = symmetric ? cost2GoPrev
                              : connector.minCost2GoConstrained(subject.state(),
                                                                node->state());
    if (cost2Go == COST_MAX) {
      continue;
    }
    float cost2RootRewire = cost2RootSubject + cost2Go;
    if (cost2RootRewire < nodeCost2Root) {
      res.emplace_back(Rewire{node, cost2Go});
    }
  }
  return res;
}

namespace {
bool contains(
    const Node &to_steer_candidate, const float *target,
    const TreeHandlerBasic::DeterministicSteerRegister &det_register) {
  auto det_register_it = det_register.find(&to_steer_candidate);
  if (det_register_it == det_register.end()) {
    return false;
  }
  return det_register_it->second.find(target) != det_register_it->second.end();
}
} // namespace

std::optional<Connector::SteerResult> extend(const View &target,
                                             TreeHandler &tree_handler,
                                             const bool is_deterministic) {
  auto *nearest = tree_handler.nearestNeighbour(target);
  auto &det_register = tree_handler.deterministic_steer_register;
  if (!nearest ||
      (is_deterministic && contains(*nearest, target.data, det_register))) {
    return std::nullopt;
  }
  if (is_deterministic) {
    det_register[nearest].emplace(target.data);
  }
  return tree_handler.problem().connector->steer(
      *nearest, target, tree_handler.parameters.steer_trials);
}

std::optional<Connector::SteerResult>
extend_star(const View &target, TreeHandler &tree_handler,
            const bool is_deterministic, std::vector<Rewire> &rewires) {
  auto maybe_steered = extend(target, tree_handler, is_deterministic);
  if (!maybe_steered) {
    return std::nullopt;
  }
  auto near_set = tree_handler.nearSet(maybe_steered->node);
  rewires = compute_rewires(maybe_steered->node, std::move(near_set),
                            DescriptionAndParameters{tree_handler.problem(),
                                                     tree_handler.parameters});
  return maybe_steered;
}

void apply_rewires_if_better(const Node &parent,
                             const std::vector<Rewire> &rewires) {
  float parentCost2Root = parent.cost2Root();
  for (const auto &rew : rewires) {
    if (parentCost2Root + rew.new_cost_from_father <
        rew.involved_node->cost2Root()) {
      rew.involved_node->setParent(parent, rew.new_cost_from_father);
    }
  }
}
} // namespace mt_rrt
