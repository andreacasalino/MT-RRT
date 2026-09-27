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
#include <MT-RRT/Sampler.h>
#include <MT-RRT/Solution.h>
#include <MT-RRT/Tree.h>

#include <deque>
#include <optional>
#include <variant>

namespace mt_rrt {
template <typename S>
concept FoundSolution = requires(const S obj_const) {
  { obj_const.costTot } -> std::same_as<float>;

  { obj_const.materialize() } -> std::same_as<Solution>;
};

template <FoundSolution S, Connector C, Sampler Smplr> class Extender {
public:
  template <FoundSolution S>
  std::optional<Solution>
  materializeBestSolution(const std::vector<S> &encoded) {
    auto it_best = std::min_element(
        encoded.begin(), encoded.end(),
        [&](const auto &a, const auto &b) { return a.costTot < b.costTot; });

    return it == encoded.end() ? std::nullopt
                               : std::make_optional(it_best->materialize());
  }

  const auto &getSolutions() const { return solutions_; }

protected:
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

  template <typename T, bool IsDeterministic>
      ExtendResult extend(std::span<const float> target,
                          T &tree) requires(tree::HasIter<T> ||
                                            tree::HasCustomQueries<T, C>) &&
      tree::HasBasicMethods<T> {
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

  void pushSolution(S to_add) { solutions_.emplace_back(std::move(to_add)); }

  std::span<const float> sampleState() const {
    sampler_.sampleState(sample_buffer_);
    return sample_buffer_;
  }

  Extender(bool isStar, Determinism det, C &connector, const Smplr &sampler);

  C &connector_;

private:
  template <typename T, Connector C>
  const Node *find_nearest_neighbour(std::span<const float> state,
                                     const T &tree, const C &connector) requires
      tree::HasIter<T> || tree::HasCustomQueries<T, C> {
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

  std::vector<S> solutions_;

  const Smplr &sampler_;
  std::vector<float> sample_buffer_;

  // scratch buffers
  std::vector<float> steer_buffer_;
  Rewiring rewiring_;
};
} // namespace mt_rrt
