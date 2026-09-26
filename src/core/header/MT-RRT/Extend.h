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
#include <variant>

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

namespace extend_result {
struct NotPossible {};
struct TargetReached {
  Positive cost2Go;
  const Node &parent;
};
struct TreeSteered {
  const Node &steered;
};
} // namespace extend_result
using ExtendResult =
    std::variant<extend_result::NotPossible, extend_result::TargetReached,
                 extend_result::TreeSteered>;

class Extender {
protected:
  Extender(bool star_extend_enabled, Determinism det);

  bool shallThisBeDeterministic() {
    return determinism_.shallThisBeDeterministic();
  }

  template <typename T, Connector C, bool IsDeterministic>
  ExtendResult extend(std::span<const float> target, T &tree,
                      C &connector) requires
      tree::HasIterOrCustomQueries<T, C> && tree::IsExtendable<T> {
    const Node *nearest = find_nearest_neighbour(target, tree, connector);
    if (!nearest) {
      return extend_result::NotPossible;
    }
    if constexpr (IsDeterministic) {
      bool is_new =
          register_.emplace(std::make_pair(nearest, target.data())).first;
      if (is_new) {
        return extend_result::NotPossible;
      }
    }

    auto traj = connector.makeTrajectory(nearest->data().state, target);
    if (!traj.has_value()) {
      return extend_result::NotPossible;
    }

    auto steer_result = traj->traverse(steer_buffer_);
    if (!steer_result.has_value()) {
      return extend_result::NotPossible;
    }
    if (steer_result->target_was_reached) {
      return extend_result::TargetReached{steer_result->cost2Go, *nearest};
    }

    const Node *steer_node =
        tree.internalize(steer_buffer_, *nearest, steer_result->cost2Go);

    if (star_extend_enabled_ && !steer_result->target_was_reached) {
      /////////////// star rewiring ///////////////
      rewiring_.update(*steer_node, tree, connector);
    }

    return extend_result::TreeSteered{.steered = *steer_node};
  }

  const auto &getRewires() const { return rewiring_.getRewires(); }

private:
  bool star_extend_enabled_{false};
  DeterminismRegulator determinism_;
  DeterministicSteerRegisterHash determinism_register_;

  // scratch buffers
  std::vector<float> steer_buffer_;
  Rewiring rewiring_;
};
} // namespace mt_rrt
