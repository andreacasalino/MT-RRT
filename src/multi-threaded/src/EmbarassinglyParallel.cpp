/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/EmbarassinglyParallel.h>
#include <MT-RRT/extender/Extender.h>

#include "MultiThreadedUtils.h"

namespace mt_rrt {
namespace {
std::vector<TreeHandlerPtr<TreeHandler>>
make_trees(const std::vector<float> &root,
           const std::vector<ProblemDescriptionPtr> &problems,
           const Parameters &pars) {
  std::vector<TreeHandlerPtr<TreeHandler>> res;
  for (auto &problem : problems) {
    res.emplace_back(std::make_unique<TreeHandler>(View{root}, problem, pars));
  }
  return res;
}
} // namespace

void EmbarassinglyParallelPlanner::solve_(const std::vector<float> &start,
                                          const std::vector<float> &end,
                                          const Parameters &parameters,
                                          PlannerSolution &recipient) {
  resizeDescriptions(getThreads());

  auto perform = [&](auto &&extenders) {
    parallel_region(getThreads(),
                    [&](std::size_t th_id) { extenders[th_id].search(); });

    recipient.iterations = parameters.iterations.get();
    recipient.solution = materialize_best_in_extenders(extenders);
    serializeTrees(extenders, parameters, recipient);
  };

  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single:
  case ExpansionStrategy::Star:
    perform(make_single_extenders(
        make_trees(start, getAllDescriptions(), parameters), end));
    break;
  case ExpansionStrategy::Bidir:
    perform(make_bidirectional_extenders(
        make_trees(start, getAllDescriptions(), parameters),
        make_trees(end, getAllDescriptions(), parameters)));
    break;
  }
}
} // namespace mt_rrt
