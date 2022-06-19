/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT-multi-threaded/ParallelizedQueriesPlanner.h>

#include <Extender.h>
#include <MultiThreadedUtils.h>
#include <ParallelFor.h>

#include <algorithm>

namespace mt_rrt {
namespace {
using ParallelForPtr = std::shared_ptr<ParallelFor>;

class ParallelQueriesTreeHandler : public TreeHandler {
public:
  ParallelQueriesTreeHandler(
      const State &root_state,
      const std::vector<ProblemDescriptionPtr> &descriptions,
      const Parameters &parameters, const ParallelForPtr &parallel_executor)
      : TreeHandler(
            make_tree([&root_state]() { return make_root(root_state); }),
            descriptions.front(), parameters),
        descriptions(descriptions) {
    parallel_for_executor = parallel_executor;
  }

  // nullptr if nothing was found
  Node *nearestNeighbour(const State &state) final {
    struct Result {
      Node *node = nullptr;
      float distance = COST_MAX;

      bool operator<(const Result &o) const { return distance < o.distance; }
    };

    auto to_reduce =
        parallel_for_executor->process<Result, Tree::const_iterator>(
            tree->begin(), tree->end(),
            [&descriptions = this->descriptions, &state](
                Result &res, const NodePtr &node, const std::size_t th_id) {
              float cost = descriptions[th_id]->connector->minCost2Go(
                  node->getState(), state);
              if (cost < res.distance) {
                res.node = node.get();
                res.distance = cost;
              }
            });

    std::sort(to_reduce.begin(), to_reduce.end());
    return to_reduce.front().node;
  }

  NearSet nearSet(const Node &subject) final {
    float ray = near_set_ray(tree->size(), tree->front()->getState().size(),
                             problem().gamma.get());

    auto to_reduce =
        parallel_for_executor->process<NearSet, Tree::const_iterator>(
            tree->begin(), tree->end(),
            [&descriptions = this->descriptions, &subject, &ray](
                NearSet &result, const NodePtr &node, const std::size_t th_id) {
              if (descriptions[th_id]->connector->minCost2Go(
                      node->getState(), subject.getState()) <= ray) {
                result.emplace(node.get(), node->cost2Root());
              }
            });

    NearSet result;
    for (const auto &to_reduce_element : to_reduce) {
      for (const auto &[node, cost] : to_reduce_element) {
        result.emplace(node, cost);
      }
    }
    auto *subject_father = subject.getFatherInfo().father;
    if (result.find(subject_father) == result.end()) {
      result.emplace(subject_father, subject_father->cost2Root());
    }
    return result;
  }

private:
  ParallelForPtr parallel_for_executor;
  std::vector<ProblemDescriptionPtr> descriptions;
};
} // namespace

void ParallelizedQueriesPlanner::solve_(const State &start, const State &end,
                                        const Parameters &parameters,
                                        PlannerSolution &recipient) {
  ParallelForPtr parallel_for_executor =
      std::make_shared<ParallelFor>(getThreads());
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
  emplace_solution(recipient, extender->getSolutions());
  recipient.trees = extender->dumpTrees();
}

} // namespace mt_rrt
