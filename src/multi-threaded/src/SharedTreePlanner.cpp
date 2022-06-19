/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT-multi-threaded/SharedTreePlanner.h>

#include <Extender.h>
#include <MultiThreadedUtils.h>

#include <algorithm>
#include <atomic>
#include <omp.h>

namespace mt_rrt {
namespace {
class NodeThreadSafe : public Node {
public:
  NodeThreadSafe(Node &&o) : Node(std::forward<Node>(o)) {}

  NodeFatherInfo getFatherInfo() const override {
    std::scoped_lock node_lock(node_mtx);
    return this->Node::getFatherInfo();
  };

  void setFatherInfo(const NodeFatherInfo &info) override {
    std::scoped_lock node_lock(node_mtx);
    this->Node::setFatherInfo(info);
  };

private:
  mutable std::mutex node_mtx;
};

NodePtr make_node_safe(Node &&o) {
  return std::make_shared<NodeThreadSafe>(std::forward<Node>(o));
}

class ThreadSafeTreeHandler : public TreeHandler {
public:
  ThreadSafeTreeHandler(const TreePtr &tree,
                        const ProblemDescriptionPtr &problem,
                        const Parameters &parameters)
      : TreeHandler(tree, problem, parameters) {}

  Node *nearestNeighbour(const State &state) final {
    Tree::const_reverse_iterator tree_begin, tree_end;
    {
      std::scoped_lock lock(expansion_mutex);
      tree_end = tree->rend();
      tree_begin = tree->rbegin();
    }
    return nearest_neighbour(state, tree_begin, tree_end,
                             DescriptionAndParameters{problem(), parameters});
  }

  NearSet nearSet(const Node &subject) final {
    const auto &connector = *problem().connector;
    Tree::const_reverse_iterator tree_begin, tree_end;
    std::size_t tree_size;
    {
      std::scoped_lock lock(expansion_mutex);
      tree_end = tree->rend();
      tree_begin = tree->rbegin();
      tree_size = tree->size();
    }
    return near_set(subject, tree_begin, tree_end,
                    DescriptionAndParameters{problem(), parameters});
  }

  Node *add(Node &&to_add) final {
    std::scoped_lock lock(expansion_mutex);
    tree->push_back(make_node_safe(std::forward<Node>(to_add)));
    return tree->back().get();
  }

private:
  mutable std::mutex expansion_mutex;
};
} // namespace

void SharedTreePlanner::solve_(const State &start, const State &end,
                               const Parameters &parameters,
                               PlannerSolution &recipient) {
  auto one_iter_parameters = parameters;
  one_iter_parameters.iterations.set(1);

  auto extenders_maker = [&](const State &state) {
    // tree is shared, while each thread uses its own dedicated problem copy
    auto tree = make_tree([&state]() { return make_node_safe(Node{state}); });
    std::vector<TreeHandlerPtr> result;
    for (std::size_t k = 0; k < this->getThreads(); ++k) {
      result.emplace_back(std::make_unique<ThreadSafeTreeHandler>(
          tree, problemAt(k), one_iter_parameters));
    }
    return result;
  };

  Extenders extenders;
  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Star:
  case ExpansionStrategy::Single: {
    auto handlers = extenders_maker(start);
    for (auto &handler : handlers) {
      extenders.emplace_back(
          std::make_unique<ExtenderSingle>(std::move(handler), end));
    }
  } break;
  case ExpansionStrategy::Bidir: {
    auto front_handlers = extenders_maker(start);
    auto back_handlers = extenders_maker(end);
    for (std::size_t k = 0; k < front_handlers.size(); ++k) {
      extenders.emplace_back(std::make_unique<ExtenderBidirectional>(
          std::move(front_handlers[k]), std::move(back_handlers[k])));
    }
  } break;
  }

  std::atomic<std::size_t> iter = 0;
  KeepSearchPredicate search_predicate{parameters.best_effort,
                                       parameters.iterations.get(),
                                       parameters.expansion_strategy};

  parallel_region(getThreads(), [&]() {
    const auto th_id = omp_get_thread_num();
    Extender &extender = *extenders[th_id];
    while (search_predicate(iter)) {
      extender.search();
      if (!extender.getSolutions().empty()) {
        search_predicate.one_solution_was_found = true;
      }
      ++iter;
    }
  });

  recipient.iterations = iter;
  emplace_solution(recipient, get_best_solution(extenders));
  recipient.trees = extenders.front()->dumpTrees();
}
} // namespace mt_rrt
