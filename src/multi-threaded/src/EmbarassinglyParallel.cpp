/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT-multi-threaded/EmbarassinglyParallel.h>

#include <Extender.h>
#include <MultiThreadedUtils.h>

namespace mt_rrt {

void EmbarassinglyParallelPlanner::solve_(const State &start, const State &end,
                                          const Parameters &parameters,
                                          PlannerSolution &recipient) {
  Extenders extenders;
  for (std::size_t t = 0; t < getThreads(); ++t) {
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
      auto tree_end = std::make_unique<TreeHandler>(make_root(end),
                                                    problemPtr(), parameters);
      extender = std::make_unique<ExtenderBidirectional>(std::move(tree_start),
                                                         std::move(tree_end));
    } break;
    }

    extenders.emplace_back(std::move(extender));
  }

  parallel_region(getThreads(),
                  [&]() { extenders[omp_get_thread_num()]->search(); });

  recipient.iterations = parameters.iterations.get();
  emplace_solution(recipient, get_best_solution(extenders));
  emplace_trees(recipient, extenders);
}
} // namespace mt_rrt
