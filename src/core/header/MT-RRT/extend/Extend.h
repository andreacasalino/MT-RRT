/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.hxx>
#include <MT-RRT/concepts/Tree.h>
#include <MT-RRT/extend/ExtendTypes.h>

#include <deque>

namespace mt_rrt {
template <Tree T, Connector C, bool IsDeterministic>
std::optional<SteerResult>
extend(std::vector<float> &reached_state, std::span<const float> target,
       T &tree, const C &connector, const SteerIterations &trials) {
  const Node *nearest = find_nearest_neighbour(target, tree.iter(), connector);
  if (!nearest) {
    return std::nullopt;
  }
  if constexpr (IsDeterministic) {
    auto &deterministic_register = tree.getDeterministicRegister();
    bool is_new =
        deterministic_register.emplace(std::make_pair(nearest, target.data()))
            .first;
    if (is_new) {
      return std::nullopt;
    }
  }
  return steer(connector, reached_state, nearest->data().state, target, trials);
}

template <Tree T, Connector C, bool IsDeterministic>
std::optional<SteerResult> extend_star(std::span<const float> target, T &tree,
                                       const C &connector,
                                       std::vector<Rewire> &rewires);
} // namespace mt_rrt
