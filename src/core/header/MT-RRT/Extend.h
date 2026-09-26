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
template <typename T, Connector C>
NearestNeighbour find_nearest_neighbour(
    std::span<const float> state, const T &tree,
    const C &connector) requires tree::HasIterOrCustomQueries<T, C> {

  // TODO first check if custom queries existing for T
  if constexpr (tree::HasIter<T>) {
    NearestNeighbour query;
    for_each_nodes(tree.iter(), [](const Node *candidate) {
      float cost2Go = connector.minCost2Go(candidate->data().state, state);
      query.update(*candidate, cost2Go);
    });
    return query;
  }

  else {
    return tree.nearestNeighbour(state, connector);
  }
}

template <typename T, Connector C>
void get_near_set(
    Rewiring &recipient, std::span<const float> state, const T &tree,
    const C &connector) requires tree::HasIterOrCustomQueries<T, C> {

  if constexpr (tree::HasIter<T>) {
    auto it = tree.iter();
    recipient.updateFirstStep(state, it.size());
    for_each_nodes(std::move(it), [](const Node *candidate) {
      recipient.update(*candidate, connector);
    });
  }

  else {
    tree.nearSet(recipient, connector);
  }
}

struct ExtendResult {
  bool target_reached{false};

  // when target_reached = true  => the node from which the extension was
  // possible
  //
  // when target_reached = false => the actually created node
  const Node *node;
};

class Extender {
protected:
  Extender(bool star_extend_enabled, const SteerIterations &trials);

  bool shallThisBeDeterministic();

  template <typename T, Connector C, bool IsDeterministic>
  std::optional<ExtendResult> extend(std::span<const float> target, T &tree,
                                     const C &connector) requires
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

    auto res =
        steer(connector, state_buffer_, nearest->data().state, target, trials_);
    if (!res.has_value()) {
      return std::nullopt;
    }

    // TODO star_extend_enabled_ ... use internal rewiring_

    if (res->target_reached) {
      return ExtendResult{true, nearest};
    }

    else {
      const Node *added = tree.internalize(
          std::span<const float>{state_buffer_}, *nearest, res->cost2Go);
      return ExtendResult{false, added};
    }
  }

private:
  SteerIterations trials_;

  // TODO determinism sampler
  bool star_extend_enabled_{false};

  DeterministicSteerRegisterHash register_;
  std::vector<float> state_buffer_;
  Rewiring rewiring_;
};
} // namespace mt_rrt
