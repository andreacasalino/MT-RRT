/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>
#include <MT-RRT/ExtendTypes.h>
#include <MT-RRT/Rewiring.h>
#include <MT-RRT/Tree.h>

#include <deque>
#include <optional>

namespace mt_rrt {
template <typename T, Connector C>
const Node *find_nearest_neighbour(
    std::span<const float> state, const T &tree,
    const C &connector) requires tree::HasIterOrCustomQueries<T, C> {

  if constexpr (tree::HasCustomQueries<T, C>) {
    return tree.nearestNeighbour(state, connector).closest;
  }

  else if constexpr (tree::HasIter<T>) {
    NearestNeighbour query{state};
    for_each_nodes(tree.iter(), [&](const Node &candidate) {
      query.update(candidate, connector);
    });
    return query.closest;
  }
}

struct DeterministicTargetReached {
  Positive cost2Go;
  const Node &parent;
};

class Extender {
protected:
  Extender(bool star_extend_enabled, Determinism det);

  bool shallThisBeDeterministic() {
    return determinism_.shallThisBeDeterministic();
  }

  template <typename T, Connector C, bool IsDeterministic>
  std::optional<DeterministicTargetReached>
  extend(std::span<const float> target, T &tree, C &connector) requires
      tree::HasIterOrCustomQueries<T, C> && tree::IsExtendable<T> {
    const Node *nearest = find_nearest_neighbour(target, tree, connector);
    if (!nearest) {
      return std::nullopt;
    }
    if constexpr (IsDeterministic) {
      bool is_new =
          register_.emplace(std::make_pair(nearest, target.data())).first;
      if (is_new) {
        return std::nullopt;
      }
    }

    auto traj = connector.makeTrajectory(nearest->data().state, target);
    if (!traj.has_value()) {
      return std::nullopt;
    }

    auto steer_result = traj->traverse(steer_buffer_);
    if (!steer_result.has_value()) {
      return std::nullopt;
    }
    if (steer_result->target_was_reached) {
      return DeterministicTargetReached{steer_result->cost2Go, *nearest};
    }

    const Node *steer_node =
        tree.internalize(steer_buffer_, *nearest, steer_result->cost2Go);

    if (star_extend_enabled_ && !steer_result->target_was_reached) {
      /////////////// star rewiring ///////////////
      rewiring_.update(*steer_node, tree, connector);

      if constexpr (tree::HasCustomRewiring<T, C>) {
        tree.applyRewiring(*steer_node, rewiring_.getRewires(), connector);
      } else {
        for (const auto &rew : rewiring_.getRewires()) {
          rew.involved_node->setParent(*steer_node, rew.updatedCost2Go);
        }
      }
    }

    return std::nullopt;
  }

private:
  bool star_extend_enabled_{false};
  DeterminismRegulator determinism_;
  DeterministicSteerRegisterHash determinism_register_;

  // scratch buffers
  std::vector<float> steer_buffer_;
  Rewiring rewiring_;
};
} // namespace mt_rrt
