/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/ParallelFor.h>
#include <MT-RRT/ParallelizedQueriesPlanner.h>
#include <MT-RRT/TreeUtils.h>
#include <MT-RRT/extender/ExtenderBidir.h>
#include <MT-RRT/extender/ExtenderSingle.h>

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
      results[threadId].process(*node, dist);
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
      results[threadId].template process<true>(*node);
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

  auto perform = [&](auto &extender) {
    recipient.iterations = extender.search();
    recipient.solution = materialize_best(extender.solutions);
    recipient.trees = extender.dumpTrees();
  };

  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single:
  case ExpansionStrategy::Star: {
    auto tree = std::make_unique<ParallelQueriesTreeHandler>(
        start, descriptions, parameters, parallel_for_executor);
    Extender<ExtenderSingle> extender{std::move(tree), end};
    perform(extender);
  } break;
  case ExpansionStrategy::Bidir: {
    auto tree_start = std::make_unique<ParallelQueriesTreeHandler>(
        start, descriptions, parameters, parallel_for_executor);
    auto tree_end = std::make_unique<ParallelQueriesTreeHandler>(
        end, descriptions, parameters, parallel_for_executor);
    Extender<ExtenderBidirectional> extender{std::move(tree_start),
                                             std::move(tree_end)};
    perform(extender);
  } break;
  }
}

} // namespace mt_rrt
