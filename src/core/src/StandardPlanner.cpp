/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/StandardPlanner.h>
#include <MT-RRT/extender/Extender.h>

namespace mt_rrt {
void StandardPlanner::solve_(const std::vector<float> &start,
                             const std::vector<float> &end,
                             const Parameters &parameters,
                             PlannerSolution &recipient) {
  auto perform = [&](auto &extender) {
    recipient.iterations = extender.search();
    recipient.solution = materialize_best(extender.solutions);
    if (parameters.dumpTrees) {
      extender.serializeTrees(recipient.trees);
    }
  };

  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single:
  case ExpansionStrategy::Star: {
    auto tree = make_tree<TreeHandler>(start, problemPtr(), parameters);
    ExtenderSingle<TreeHandler> extender{std::move(tree), end};
    perform(extender);
  } break;
  case ExpansionStrategy::Bidir: {
    auto tree_start = make_tree<TreeHandler>(start, problemPtr(), parameters);
    auto tree_end = make_tree<TreeHandler>(end, problemPtr(), parameters);
    ExtenderBidirectional<TreeHandler> extender{std::move(tree_start),
                                                          std::move(tree_end)};
    perform(extender);
  } break;
  }
}
} // namespace mt_rrt
