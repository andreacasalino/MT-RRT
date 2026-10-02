/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>
#include <MT-RRT/ExtendTypes.h>
#include <MT-RRT/ProblemDescription.h>
#include <MT-RRT/Rewiring.h>
#include <MT-RRT/Sampler.h>
#include <MT-RRT/Solution.h>
#include <MT-RRT/Tree.h>
#include <MT-RRT/Types.h>

#include <deque>
#include <optional>
#include <variant>

namespace mt_rrt {
template <typename S>
concept FoundSolution = requires(const S obj_const) {
  { obj_const.costTot } -> std::same_as<float>;

  { obj_const.materialize() } -> std::same_as<Solution>;
};

template <typename P, FoundSolution S> class Extender {
public:
  std::optional<Solution> materializeBestSolution() const;

  const auto &getSolutions() const { return solutions_; }

  bool hasSolution() const { return !solutions_.empty(); }

protected:
  template <typename T>
  const Node *find_nearest_neighbour(std::span<const float> state,
                                     const T &tree) requires tree::HasIter<T> ||
      tree::HasCustomQueries<T, typename P::connector_type>;

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
      ExtendResult extend(std::span<const float> target, T &tree) requires(
          tree::HasIter<T> ||
          tree::HasCustomQueries<T, typename P::connector_type>) &&
      tree::HasBasicMethods<T>;

  void pushSolution(S &&to_add) {
    solutions_.emplace_back(std::forward<S>(to_add));
  }

  std::span<const float> sampleState() const {
    problem.sampler.sampleState(sample_buffer_);
    return sample_buffer_;
  }

  Extender(P &prblm) : problem{prblm} {}

  P &problem;

private:
  DeterministicSteerRegisterHash determinism_register_;

  std::vector<S> solutions_;

  // scratch buffers
  std::vector<float> sample_buffer_;
  std::vector<float> steer_buffer_;
  Rewiring rewiring_;
};

template <typename E>
concept ConcreteExtender = requires(E obj, const E obj_const) {
  { obj.extend() } -> std::same_as<void>;

  { obj_const.hasSolution() } -> std::same_as<bool>;
};

template <ConcreteExtender E>
std::size_t extend_many(E &ext,
                        std::shared_ptr<KeepSearchPredicate> search_predicate) {
  std::size_t iter = 0;
  for (; search_predicate->keepSearch(iter); ++iter) {
    ext.extend();
    search_predicate->one_solution_was_found.store(ext.hasSolution(),
                                                   std::memory_order::release);
  }
  return iter;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <typename P, FoundSolution S>
std::optional<Solution> Extender<P, S>::materializeBestSolution() const {
  auto it_best = std::min_element(
      solutions_.begin(), solutions_.end(),
      [&](const auto &a, const auto &b) { return a.costTot < b.costTot; });

  return it == solutions_.end() ? std::nullopt
                                : std::make_optional(it_best->materialize());
}

template <typename P, FoundSolution S>
template <typename T>
const Node *
Extender<P, S>::find_nearest_neighbour<T>(std::span<const float> state,
                                          const T &tree) requires
    tree::HasIter<T> || tree::HasCustomQueries<T, typename P::connector_type> {
  if constexpr (tree::HasCustomQueries<T, typename P::connector_type>) {
    return tree.nearestNeighbour(state, *problem.connector).closest;
  }

  else if constexpr (tree::HasIter<T>) {
    NearestNeighbour query{state};
    for_each_nodes(tree.iter(), [&](const Node &candidate) {
      query.update(candidate, *problem.connector);
    });
    return query.closest;
  }
}

template <typename P, FoundSolution S>
    template <typename T, bool IsDeterministic>
    Extender<P, S>::ExtendResult
    Extender<P, S>::extend<T>(std::span<const float> target, T &tree) requires(
        tree::HasIter<T> ||
        tree::HasCustomQueries<T, typename P::connector_type>) &&
    tree::HasBasicMethods<T> {
  const Node *nearest =
      find_nearest_neighbour(target, tree, *problem.connector);
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

  auto steer_result =
      problem.connector->steer(nearest->data().state, target, steer_buffer_);
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
    rewiring_.update(*steer_node, tree, *problem.connector);

    if constexpr (tree::HasCustomRewiring<T, typename P::connector_type>) {
      tree.applyRewiring(*steer_node, rewiring_.getRewires(),
                         *problem.connector);
    } else {
      for (const auto &rew : rewiring_.getRewires()) {
        rew.involved_node->setParent(*steer_node, rew.updatedCost2Go);
      }
    }
  }

  return Steered{*steer_node};
}
} // namespace mt_rrt
