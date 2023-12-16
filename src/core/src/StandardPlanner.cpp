/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/ExtenderBidir.h>
#include <MT-RRT/ExtenderSingle.h>
#include <MT-RRT/StandardPlanner.h>

namespace mt_rrt {
void StandardPlanner::solve_(const std::vector<float> &start,
                             const std::vector<float> &end,
                             const Parameters &parameters,
                             PlannerSolution &recipient) {
  ExtenderPtr extender;
  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single:
  case ExpansionStrategy::Star: {
    auto tree =
        std::make_unique<TreeHandlerBasic>(start, problemPtr(), parameters);
    extender = std::make_unique<ExtenderSingle>(std::move(tree), end);
  } break;
  case ExpansionStrategy::Bidir: {
    auto tree_start =
        std::make_unique<TreeHandlerBasic>(start, problemPtr(), parameters);
    auto tree_end =
        std::make_unique<TreeHandlerBasic>(end, problemPtr(), parameters);
    extender = std::make_unique<ExtenderBidirectional>(std::move(tree_start),
                                                       std::move(tree_end));
  } break;
  }

  recipient.iterations = extender->search();
  recipient.solution = find_best_solution(extender->getSolutions());
  recipient.trees = extender->dumpTrees();
}
} // namespace mt_rrt
