/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Tree.h>
#include <MT-RRT/Utils.h>

namespace mt_rrt {
template <Connector C, typename T>
void Rewiring::update(Node &pivot, const T &tree, C &connector) {
  rewires_.clear();
  NearSetHandler handler{nearSetRay(tree.size()), pivot, near_set_};

  if constexpr (tree::HasCustomNearSet<T, C>) {
    tree.nearSet(handler, connector);
  } else {
    for_each_nodes(tree.iter(),
                   [&](const Node &next) { handler.tryAdd(next, connector); });
  }

  computeRewires(handler, connector);
}

template <Connector C>
void Rewiring::computeRewires(NearSetHandler &handler, C &connector) {
  auto it_best = std::min_element(
      handler.near_set.begin(), handler.near_set.end(),
      [](const auto &a, const auto &b) { return a.costTot() < b.costTot(); });
  if (it_best == handler.near_set.end()) {
    return;
  }

  // rewire just_steered to the best father
  handler.pivot.setParent(*it_best->element, it_best->cost2go);
  float cost2RootPivot = it_best->costTot();
  // remove current parent from rewire candidates
  handler.near_set.erase(it_best);

  // check for rewires
  std::vector<Rewire> res;
  for (auto [isRoot, node, nodeCost2Root, cost2GoPrev] : handler.near_set) {
    if (isRoot) {
      // root can't be rewired
      continue;
    }
    float cost2RootRewired = cost2RootPivot + cost2GoPrev;
    if (cost2RootRewired < nodeCost2Root) {
      res.emplace_back(Rewire{node, cost2GoPrev});
    }
  }
}
} // namespace mt_rrt
