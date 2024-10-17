/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/EmbarassinglyParallel.h>
#include <MT-RRT/extender/ExtenderBidir.h>
#include <MT-RRT/extender/ExtenderSingle.h>

#include "MultiThreadedUtils.h"

namespace mt_rrt {

void EmbarassinglyParallelPlanner::solve_(const std::vector<float> &start,
                                          const std::vector<float> &end,
                                          const Parameters &parameters,
                                          PlannerSolution &recipient) {
  auto perform = [&](auto &extenders) {
    parallel_region(getThreads(),
                    [&]() { extenders[omp_get_thread_num()].search(); });

    recipient.iterations = parameters.iterations.get();
    recipient.solution = materialize_best_in_extenders(extenders);
    emplace_trees(recipient, extenders);
  };

  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single:
  case ExpansionStrategy::Star: {
    Extenders<ExtenderSingle> extenders;
    for (std::size_t t = 0; t < getThreads(); ++t) {
      auto tree = std::make_unique<TreeHandlerBasic>(View{start}, problemPtr(),
                                                     parameters);
      extenders.emplace_back(std::move(tree), end);
      perform(extenders);
    }
  } break;
  case ExpansionStrategy::Bidir: {
    Extenders<ExtenderBidirectional> extenders;
    for (std::size_t t = 0; t < getThreads(); ++t) {
      auto tree_start = std::make_unique<TreeHandlerBasic>(
          View{start}, problemPtr(), parameters);
      auto tree_end = std::make_unique<TreeHandlerBasic>(
          View{end}, problemPtr(), parameters);
      extenders.emplace_back(std::move(tree_start), std::move(tree_end));
      perform(extenders);
    }
    break;
  }
  }
} // namespace mt_rrt
