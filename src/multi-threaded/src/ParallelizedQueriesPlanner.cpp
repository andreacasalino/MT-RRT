/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/ExtenderBidir.h>
#include <MT-RRT/ExtenderSingle.h>
#include <MT-RRT/ExtenderUtils.h>
#include <MT-RRT/ParallelFor.h>
#include <MT-RRT/ParallelizedQueriesPlanner.h>

#include "MultiThreadedUtils.h"

#include <algorithm>

namespace mt_rrt {
namespace {
class ParallelQueriesTreeHandler : public TreeHandlerBasic {
public:
  ParallelQueriesTreeHandler(
      const View &rootState,
      const std::vector<ProblemDescriptionPtr> &descriptions,
      const Parameters &parameters, ParallelFor &parallelFor)
      : TreeHandlerBasic(rootState, descriptions.front(), parameters),
        parallelFor{&parallelFor}, descriptions(descriptions) {}

  // nullptr if nothing was found
  const Node *nearestNeighbour(const View &state) const override {
    std::vector<NearestQuery> results;
    results.resize(parallelFor->size());
    parallelFor->process(nodes, [&](const Node *node, std::size_t threadId) {
      float dist =
          descriptions[threadId]->connector->minCost2Go(node->state(), state);
      results[threadId](*node, dist);
    });
    return std::min_element(results.begin(), results.end(),
                            [](const NearestQuery &a, const NearestQuery &b) {
                              return a.closestCost < b.closestCost;
                            })
        ->closest;
  }

  NearSet nearSet(const Node &subject) const override {
    float ray = near_set_ray(nodes.size(), nodes.front()->state().size,
                             problem().gamma.get());
    std::vector<NearSetQuery> results;
    for (std::size_t k = 0; k < descriptions.size(); ++k) {
      results.emplace_back(
          NearSetQuery{ray, subject.state(), descriptions[k]->connector.get()});
    }
    results.resize(parallelFor->size());
    parallelFor->process(nodes, [&](Node *node, std::size_t threadId) {
      results[threadId](*node);
    });
    NearSet res;
    res.cost2RootSubject = subject.cost2Root();
    for (const auto &r : results) {
      res.set.insert(res.set.end(), r.set.begin(), r.set.end());
    }
    return res;
  }

private:
  ParallelFor *parallelFor;
  std::vector<ProblemDescriptionPtr> descriptions;
};
} // namespace

void ParallelizedQueriesPlanner::solve_(const std::vector<float> &start,
                                        const std::vector<float> &end,
                                        const Parameters &parameters,
                                        PlannerSolution &recipient) {
  ParallelFor parallel_for_executor{getThreads()};
  resizeDescriptions(getThreads());
  const auto &descriptions = getAllDescriptions();

  ExtenderPtr extender;
  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single:
  case ExpansionStrategy::Star: {
    auto tree = std::make_unique<ParallelQueriesTreeHandler>(
        start, descriptions, parameters, parallel_for_executor);
    extender = std::make_unique<ExtenderSingle>(std::move(tree), end);
  } break;
  case ExpansionStrategy::Bidir: {
    auto tree_start = std::make_unique<ParallelQueriesTreeHandler>(
        start, descriptions, parameters, parallel_for_executor);
    auto tree_end = std::make_unique<ParallelQueriesTreeHandler>(
        end, descriptions, parameters, parallel_for_executor);
    extender = std::make_unique<ExtenderBidirectional>(std::move(tree_start),
                                                       std::move(tree_end));
  } break;
  }

  recipient.iterations = extender->search();
  recipient.solution = find_best_solution(extender->getSolutions());
  recipient.trees = extender->dumpTrees();
}

} // namespace mt_rrt
