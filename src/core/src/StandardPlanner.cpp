/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/extender/ExtenderSingle.h>
#include <MT-RRT/extender/ExtenderBidir.h>
#include <MT-RRT/extender/Extender.h>
#include <MT-RRT/StandardPlanner.h>

namespace mt_rrt {
void StandardPlanner::solve_(const std::vector<float> &start,
                             const std::vector<float> &end,
                             const Parameters &parameters,
                             PlannerSolution &recipient) {
  auto perform = [&recipient](auto& extender) {
    recipient.iterations = extender.search();
    recipient.solution = materialize_best(extender.solutions);
    recipient.trees = extender.dumpTrees();
  };

  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single:
  case ExpansionStrategy::Star: {
    auto tree =
        std::make_unique<TreeHandlerBasic>(start, problemPtr(), parameters);
    Extender<ExtenderSingle> extender{std::move(tree), end};
    perform(extender);
  } break;
  case ExpansionStrategy::Bidir: {
    auto tree_start =
        std::make_unique<TreeHandlerBasic>(start, problemPtr(), parameters);
    auto tree_end =
        std::make_unique<TreeHandlerBasic>(end, problemPtr(), parameters);
    Extender<ExtenderBidirectional> extender{std::move(tree_start), std::move(tree_end)};
    perform(extender);
  } break;
  }
}
} // namespace mt_rrt
