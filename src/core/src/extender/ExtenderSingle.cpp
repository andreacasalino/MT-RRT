/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/extender/ExtenderSingle.h>
#include <MT-RRT/extender/Utils.h>

namespace mt_rrt {
std::vector<std::vector<float>> SimpleSolution::materialize() const {
  auto result = sequence_from_root(*byPassNode);
  result.emplace_back(target.convert());
  return result;
}

float SimpleSolution::cost() const {
  return byPassNode->cost2Root() + cost2Target;
}

ExtenderSingle::ExtenderSingle(TreeHandlerPtr handler,
                               const std::vector<float> &trgt)
    : ProblemAware(*handler), target(trgt), tree_handler{std::move(handler)} {}

void ExtenderSingle::search_iteration(Solutions<SimpleSolution> &solutions,
                                      bool deterministic) {
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
    solutions.emplace_back(std::make_shared<SimpleSolution>(
        steered.getParent(), cost2Target, target));
    return;
  }

  auto *added = tree_handler->internalize(steered);
  if ((nullptr == added) || (rewires.involved_nodes.empty())) {
    return;
  }
  tree_handler->applyRewires(*added, rewires);
}
} // namespace mt_rrt
