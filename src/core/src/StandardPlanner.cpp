/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT-core/StandardPlanner.h>

#include <Extender.h>

namespace mt_rrt {
void StandardPlanner::solve_(const State &start, const State &end,
                             const Parameters &parameters,
                             PlannerSolution &recipient) {
  ExtenderPtr extender;

  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single:
  case ExpansionStrategy::Star: {
    auto tree = std::make_unique<TreeHandler>(make_root(start), problemPtr(),
                                              parameters);
    extender = std::make_unique<ExtenderSingle>(std::move(tree), end);
  } break;
  case ExpansionStrategy::Bidir: {
    auto tree_start = std::make_unique<TreeHandler>(make_root(start),
                                                    problemPtr(), parameters);
    auto tree_end =
        std::make_unique<TreeHandler>(make_root(end), problemPtr(), parameters);
    extender = std::make_unique<ExtenderBidirectional>(std::move(tree_start),
                                                       std::move(tree_end));
  } break;
  }

  recipient.iterations = extender->search();
  emplace_solution(recipient, extender->getSolutions());
  recipient.trees = extender->dumpTrees();
}
} // namespace mt_rrt
