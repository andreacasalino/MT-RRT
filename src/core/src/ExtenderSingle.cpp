/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/ExtenderSingle.h>

#include <MT-RRT/ExtenderUtils.h>

namespace mt_rrt {
std::vector<std::vector<float>> SimpleSolution::getSequence() const {
  auto result = sequence_from_root(*byPassNode);
  result.emplace_back(target.convert());
  return result;
}

float SimpleSolution::cost() const {
  return byPassNode->cost2Root() + cost2Target;
}

ExtenderSingle::ExtenderSingle(TreeHandlerPtr handler,
                               const std::vector<float> &target)
    : Extender(*handler), target(target), tree_handler{std::move(handler)} {}

void ExtenderSingle::search_iteration() {
  bool toward_target = determinism_manager->doDeterministicExtension();

  std::vector<float> sampled_state;
  if (!toward_target) {
    sampled_state = problem().sampler->sampleState();
  }
  View target_state = toward_target ? View{target} : View{sampled_state};

  std::optional<Connector::SteerResult> maybe_steered;
  std::vector<Rewire> rewires;
  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single:
    maybe_steered = extend(target_state, *tree_handler, toward_target);
    break;
  case ExpansionStrategy::Star:
    maybe_steered =
        extend_star(target_state, *tree_handler, toward_target, rewires);
    break;
  default:
    throw Error{"Trying to use ExtenderSingle with a bidirectional strategy"};
    break;
  }

  if (!maybe_steered) {
    return;
  }

  const auto &[steered, target_is_reached] = maybe_steered.value();
  if (toward_target && maybe_steered->target_is_reached) {
    float cost2Target = steered.cost2Go();
    solutions.emplace_back(std::make_shared<SimpleSolution>(
        steered.getParent(), cost2Target, target));
    return;
  }

  auto *added = tree_handler->internalize(steered);
  if ((nullptr == added) || (rewires.empty())) {
    return;
  }
  tree_handler->applyRewires(*added, rewires);
}
} // namespace mt_rrt
