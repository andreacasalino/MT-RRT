/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/SharedTreePlanner.h>
#include <MT-RRT/extender/Extender.h>
#include <MT-RRT/LockFreeForwardList.h>
#include <MT-RRT/SpinLock.h>

#include "MultiThreadedUtils.h"
#include <MT-RRT/TreeUtils.h>

#include <omp.h>

namespace mt_rrt {
namespace {
class SharedTreeHandler : public TreeHandler {
public:
  const Node *nearestNeighbour(const View &state) const {
    const auto &connector = *problem().connector;
    NearestQuery result;
    shared->nodes.forEach([&](Node *node) {
      result.process(*node, connector.minCost2Go(node->state(), state));
    });
    return result.closest;
  }

  NearSet nearSet(const Node &node) const {
    NearSet res;
    std::size_t problem_size = shared->root.state().size;
    float ray = near_set_ray(shared->nodes.size(), problem_size, problem().gamma.get());
    NearSetQuery query{ray, node.state(), problem().connector.get()};
    shared->nodes.forEach([&](Node *node) { query.template process<false>(*node); });
    res.set = std::move(query.set);

    SpinLockGuard guard{shared->lock};
    res.cost2RootSubject = node.cost2Root();
    for(auto& node : res.set) {
      node.cost2Root = node.element->cost2Root();
    }
    return res;
  }

  Node *internalize(const Node &subject) {
    auto *added = &shared->allocators[threadId].emplace_back(subject.state());
    added->setParent(*subject.getParent(), subject.cost2Go());
    shared->nodes.emplace_back(added);
    return added;
  }

  void applyRewires(const Node &parent, const Rewires &rewires) {
    SpinLockGuard guard{shared->lock};
    for (const auto &rewire : rewires.involved_nodes) {
      rewire.node->setParent(parent, rewire.new_cost_from_father);
    }
  }

  /////////////////////////////////////////////////////////////////////////////

  static std::vector<std::unique_ptr<SharedTreeHandler>>
  make_trees(const View &root,
             const std::vector<ProblemDescriptionPtr> &problems,
             const Parameters &parameters) {
    std::vector<std::unique_ptr<SharedTreeHandler>> res;
    auto shared = std::make_shared<SharedData>(root, problems.size());
    for (std::size_t th_id = 0; th_id < problems.size(); ++th_id) {
      res.emplace_back().reset(
          new SharedTreeHandler(problems[th_id], parameters, shared, th_id));
    }
    return res;
  }

  void copyToVectorNodes() {
    shared->nodes.forEach(
        [&nodes = this->nodes](Node *n) { nodes.push_back(n); });
    nodes.erase(nodes.begin());
  }

private:
  struct SharedData {
    SharedData(const View &root_view, std::size_t threads)
    : root{root_view}, nodes{&root} {
      allocators.resize(threads);
    };

    NodeOwning root;
    LockFreeForwardList<Node *> nodes;
    std::vector<NodesAllocator> allocators;
    SpinLock lock;
  };

  SharedTreeHandler(const ProblemDescriptionPtr &problem,
                    const Parameters &parameters,
                    std::shared_ptr<SharedData> shared, std::size_t threadId)
      : TreeHandler(problem, parameters), threadId{threadId}, shared{shared} {
    this->parameters.iterations.set(1);
    nodes.push_back(&shared->root);
  }

  std::size_t threadId;
  std::shared_ptr<SharedData> shared;
};
} // namespace

void SharedTreePlanner::solve_(const std::vector<float> &start,
                               const std::vector<float> &end,
                               const Parameters &parameters,
                               PlannerSolution &recipient) {
  resizeDescriptions(getThreads());

  std::vector<SharedTreeHandler *> handlers;
  auto perform = [&](auto &extenders) {
    std::atomic<std::size_t> iter = 0;
    KeepSearchPredicate search_predicate{parameters.best_effort,
                                        parameters.iterations.get(),
                                        parameters.expansion_strategy};

    parallel_region(getThreads(), [&]() {
      const auto th_id = omp_get_thread_num();
      auto &extender = extenders[th_id];
      while (search_predicate(iter.load(std::memory_order::memory_order_acquire))) {
        iter += extender.search();
        if (!extender.solutions.empty()) {
          search_predicate.one_solution_was_found.store(true, std::memory_order::memory_order_release);
        }
      }
    });

    recipient.iterations = iter;
    recipient.solution = materialize_best_in_extenders(extenders);
    for (auto *tree : handlers) {
      tree->copyToVectorNodes();
    }
    for (auto &&tree : extenders.front().dumpTrees()) {
      recipient.trees.emplace_back(std::move(tree));
    }
  };

  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single:
  case ExpansionStrategy::Star: {
    auto trees =
        SharedTreeHandler::make_trees(start, getAllDescriptions(), parameters);
    handlers.push_back(trees.front().get());
    Extenders<ExtenderSingle> extenders;
    for (auto &tree : trees) {
      extenders.emplace_back(std::move(tree), end);
    }
    perform(extenders);
  } break;
  case ExpansionStrategy::Bidir: {
    auto front_trees =
        SharedTreeHandler::make_trees(start, getAllDescriptions(), parameters);
    handlers.push_back(front_trees.front().get());
    auto back_trees =
        SharedTreeHandler::make_trees(end, getAllDescriptions(), parameters);
    handlers.push_back(back_trees.front().get());
    Extenders<ExtenderBidirectional> extenders;
    for (std::size_t k = 0; k < front_trees.size(); ++k) {
      extenders.emplace_back(std::move(front_trees[k]), std::move(back_trees[k]));
    }
    perform(extenders);
  } break;
  }
}
} // namespace mt_rrt
