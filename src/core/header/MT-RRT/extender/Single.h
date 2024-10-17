/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Planner.h>
#include <MT-RRT/ProblemDescription.h>
#include <MT-RRT/Solution.h>
#include <MT-RRT/TreeHandler.h>
#include <MT-RRT/TreeUtils.h>
#include <MT-RRT/extender/Types.h>

namespace mt_rrt::extender {
struct SimpleSolution {
  std::vector<std::vector<float>> materialize() const;

  float cost() const { return byPassNode->cost2Root() + cost2Target; }

  const Node *byPassNode;
  float cost2Target;
  View target;
};

template <typename TreeT> class Single : public ProblemAware {
public:
  using SolutionT = SimpleSolution;

  std::vector<float> target;
  TreeHandlerPtr<TreeT> tree_handler;

  Single(TreeHandlerPtr<TreeT> handler,
                 const std::vector<float> &target);

  void search_iteration(Solutions<SimpleSolution> &solutions,
                        bool deterministic);

  void serializeTrees(
      std::vector<PlannerSolution::TreeSerialized> &recipient) const {
    recipient.emplace_back(serialize_tree(tree_handler->nodes));
  }

  const Parameters &parameters() const { return tree_handler->parameters; }
};

//////////////////////////////////////////////////////////////////

template <typename TreeT>
Single<TreeT>::Single(TreeHandlerPtr<TreeT> handler,
                                      const std::vector<float> &trgt)
    : ProblemAware(*handler), target(trgt), tree_handler{std::move(handler)} {}

template <typename TreeT>
void Single<TreeT>::search_iteration(
    Solutions<SimpleSolution> &solutions, bool deterministic) {
  std::vector<float> sampled_state;
  if (!deterministic) {
    sampled_state = problem().sampler->sampleState();
  }
  View target_state = deterministic ? View{target} : View{sampled_state};

  std::optional<Connector::SteerResult> maybe_steered;
  Rewires rewires;
  switch (parameters().expansion_strategy) {
  case ExpansionStrategy::Single:
    maybe_steered = extend(target_state, *tree_handler, deterministic);
    break;
  case ExpansionStrategy::Star:
    maybe_steered =
        extend_star(target_state, *tree_handler, deterministic, rewires);
    break;
  default:
    throw Error{"Trying to use ExtenderSingle with a bidirectional strategy"};
    break;
  }

  if (!maybe_steered) {
    return;
  }

  const auto &[target_is_reached, steered] = maybe_steered.value();
  if (deterministic && maybe_steered->target_is_reached) {
    float cost2Target = steered.cost2Go();
    solutions.emplace_back(
        SimpleSolution{steered.getParent(), cost2Target, target});
    return;
  }

  auto *added = tree_handler->internalize(steered);
  if ((nullptr == added) || (rewires.involved_nodes.empty())) {
    return;
  }
  tree_handler->applyRewires(*added, rewires);
}

} // namespace mt_rrt
