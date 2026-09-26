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
#include <variant>

namespace mt_rrt {
class Extender {
protected:
  Extender(bool isStar, Determinism det);

  bool shallThisBeDeterministic() {
    return determinism_.shallThisBeDeterministic();
  }

  struct ExtendNotPossible {};
  struct DeterministicTargetReached {
    Positive cost2Go;
    const Node &parent;
  };
  struct Steered {
    const Node &added;
  };
  using ExtendResult =
      std::variant<ExtendNotPossible, DeterministicTargetReached, Steered>;

  template <typename T, Connector C, bool IsDeterministic>
  ExtendResult extend(std::span<const float> target, T &tree,
                      C &connector) requires
      tree::HasIterOrCustomQueries<T, C> && tree::IsTree<T> {
    const Node *nearest = find_nearest_neighbour(target, tree, connector);
    if (!nearest) {
      return ExtendNotPossible;
    }
    if constexpr (IsDeterministic) {
      bool is_new =
          register_.emplace(std::make_pair(nearest, target.data())).second;
      if (is_new) {
        return ExtendNotPossible;
      }
    }

    auto traj = connector.makeTrajectory(nearest->data().state, target);
    if (!traj.has_value()) {
      return ExtendNotPossible;
    }

    auto steer_result = traj->traverse(steer_buffer_);
    if (!steer_result.has_value()) {
      return ExtendNotPossible;
    }
    if (steer_result->target_was_reached) {
      return DeterministicTargetReached{steer_result->cost2Go, *nearest};
    }

    const Node *steer_node =
        tree.internalize(steer_buffer_, *nearest, steer_result->cost2Go);

    if (isStar_ && !steer_result->target_was_reached) {
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

    return Steered{*steer_node};
  }

private:
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

  bool isStar_{false};
  DeterminismRegulator determinism_;
  DeterministicSteerRegisterHash determinism_register_;

  // scratch buffers
  std::vector<float> steer_buffer_;
  Rewiring rewiring_;
};
} // namespace mt_rrt
