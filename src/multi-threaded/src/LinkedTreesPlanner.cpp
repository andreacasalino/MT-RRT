/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT-multi-threaded/LinkedTreesPlanner.h>

#include <Extender.h>
#include <LinkedCache.h>
#include <MT-RRT-carpet/Error.h>
#include <MultiThreadedUtils.h>

#include <algorithm>
#include <math.h>
#include <omp.h>

namespace mt_rrt {
namespace {
class LinkedTreeHandler : public TreeHandler, public LinkedCache<NodePtr> {
public:
  LinkedTreeHandler(const NodePtr &root, const ProblemDescriptionPtr &problem,
                    const Parameters &parameters)
      : TreeHandler(root, problem, parameters) {}

  Node *add(Node &&to_add) override {
    for (auto &cache : LinkedCache<NodePtr>::outgoing_caches) {
      NodePtr clone = std::make_shared<Node>(to_add.getState());
      clone->setFatherInfo(to_add.getFatherInfo());
      cache->push_back(clone);
    }
    return this->TreeHandler::add(std::forward<Node>(to_add));
  }

  virtual void drainCaches() {
    LinkedCache<NodePtr>::drainIncomingCaches(
        [&tree = this->tree](const NodePtr &to_add) {
          tree->push_back(to_add);
        });
  }
};
using LinkedTreeHandlerPtr = std::unique_ptr<LinkedTreeHandler>;

std::vector<LinkedTreeHandlerPtr>
make_linked_trees(const State &root_state,
                  const std::vector<ProblemDescriptionPtr> &problem_clones,
                  const Parameters &parameters) {
  std::vector<LinkedTreeHandlerPtr> result;
  for (const auto &problem : problem_clones) {
    result.emplace_back(std::make_unique<LinkedTreeHandler>(
        make_root(root_state), problem, parameters));
  }
  LinkedCache<NodePtr>::link_caches(result.begin(), result.end());
  return result;
}

class LinkedNode;
using LinkedNodePtr = std::shared_ptr<LinkedNode>;
class LinkedNode : public Node {
public:
  static std::vector<LinkedNodePtr>
  make_roots(const State &root_state, const std::size_t clones_to_generate) {
    std::vector<LinkedNodePtr> roots;
    for (std::size_t k = 0; k < clones_to_generate; ++k) {
      roots.emplace_back().reset(new LinkedNode(root_state));
    }
    link(roots);
    return roots;
  }

  static LinkedNodePtr make_linked_node(Node &&o,
                                        const std::size_t calling_thread) {
    auto o_father_info = o.getFatherInfo();
    std::vector<LinkedNodePtr> linked_nodes;
    const auto &father_clones =
        static_cast<LinkedNode *>(o_father_info.father)->getLinked();
    auto father_clones_it = father_clones.begin();
    const size_t threads = father_clones.size() + 1;
    for (std::size_t k = 0; k < threads; ++k) {
      if (k == calling_thread) {
        linked_nodes.emplace_back();
      } else {
        linked_nodes.emplace_back().reset(new LinkedNode{
            o.getState(), **father_clones_it, o_father_info.cost_from_father});
        ++father_clones_it;
      }
    }
    linked_nodes[calling_thread].reset(new LinkedNode{std::forward<Node>(o)});
    link(linked_nodes);
    return linked_nodes[calling_thread];
  }

  const std::vector<LinkedNodePtr> &getLinked() const { return linked; }

private:
  LinkedNode(const State &root_state) : Node(root_state) {}

  LinkedNode(Node &&n) : Node(std::forward<Node>(n)) {}
  LinkedNode(const State &state, Node &father, const float cost_from_father)
      : LinkedNode(state) {
    setFatherInfo(NodeFatherInfo{&father, cost_from_father});
  }

  static void link(const std::vector<LinkedNodePtr> nodes) {
    for (std::size_t k = 0; k < nodes.size(); ++k) {
      auto &linked = nodes[k]->linked;
      for (LinkedIndicesRange l(k); l < nodes.size(); ++l) {
        linked.push_back(nodes[l]);
      }
    }
  }

  std::vector<LinkedNodePtr> linked;
};

struct RewireComplete {
  Node &new_father;
  Rewire info;
};

class LinkedStarTreeHandler : public LinkedTreeHandler,
                              public LinkedCache<RewireComplete> {
public:
  LinkedStarTreeHandler(const LinkedNodePtr &root,
                        const ProblemDescriptionPtr &problem,
                        const Parameters &parameters)
      : LinkedTreeHandler(root, problem, parameters),
        thread_numb(static_cast<std::size_t>(omp_get_thread_num())) {}

  Node *add(Node &&to_add) final {
    auto as_linked_node =
        LinkedNode::make_linked_node(std::forward<Node>(to_add), thread_numb);
    const auto &linked_nodes = as_linked_node->getLinked();
    for (std::size_t o = 0; o < linked_nodes.size(); ++o) {
      LinkedCache<NodePtr>::outgoing_caches[0]->push_back(linked_nodes[o]);
    }
    tree->push_back(as_linked_node);
    return tree->back().get();
  }

  void applyRewires(Node &new_father, const Rewires &rewires) final {
    const auto &father_clones =
        static_cast<LinkedNode &>(new_father).getLinked();
    for (const auto &[involved_node, new_cost] : rewires) {
      const auto &involved_clones =
          static_cast<LinkedNode &>(involved_node).getLinked();
      for (std::size_t k = 0; k < involved_clones.size(); ++k) {
        this->LinkedCache<RewireComplete>::outgoing_caches[k]->push_back(
            RewireComplete{*father_clones[k],
                           Rewire{*involved_clones[k], new_cost}});
      }
    }
    this->TreeHandler::applyRewires(new_father, rewires);
  }

  void drainCaches() override {
    this->LinkedTreeHandler::drainCaches();
    LinkedCache<RewireComplete>::drainIncomingCaches(
        [](const RewireComplete &rewire) {
          rewire.info.involved_node.setFatherInfo(NodeFatherInfo{
              &rewire.new_father, rewire.info.new_cost_from_father});
        });
  }

private:
  const std::size_t thread_numb;
};

std::vector<LinkedTreeHandlerPtr>
make_linked_star_trees(const State &root_state,
                       const std::vector<ProblemDescriptionPtr> &problem_clones,
                       const Parameters &parameters) {
  std::vector<LinkedTreeHandlerPtr> result;
  auto roots = LinkedNode::make_roots(root_state, problem_clones.size());
  for (std::size_t k = 0; k < roots.size(); ++k) {
    result.emplace_back(std::make_unique<LinkedStarTreeHandler>(
        roots[k], problem_clones[k], parameters));
  }
  LinkedCache<NodePtr>::link_caches(result.begin(), result.end());
  LinkedCache<RewireComplete>::link_caches(result.begin(), result.end());
  return result;
}
} // namespace

void LinkedTreesPlanner::solve_(const State &start, const State &end,
                                const Parameters &parameters,
                                PlannerSolution &recipient) {
  Extenders extenders;

  std::vector<LinkedTreeHandler *> registered_handlers;
  auto register_handlers =
      [&registered_handlers](
          const std::vector<LinkedTreeHandlerPtr> &handlers) {
        for (const auto &element : handlers) {
          registered_handlers.push_back(element.get());
        }
      };

  resizeDescriptions(getThreads());

  auto batched_iterations = compute_batched_iterations(
      parameters.iterations, getThreads(), synchronization());

  auto batch_iter_parameters = parameters;
  batch_iter_parameters.iterations.set(batched_iterations);

  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single: {
    auto handlers =
        make_linked_trees(start, getAllDescriptions(), batch_iter_parameters);
    register_handlers(handlers);
    for (auto &handler : handlers) {
      extenders.emplace_back(
          std::make_unique<ExtenderSingle>(std::move(handler), end));
    }
  } break;
  case ExpansionStrategy::Bidir: {
    auto front_handlers =
        make_linked_trees(start, getAllDescriptions(), batch_iter_parameters);
    register_handlers(front_handlers);
    auto back_handlers =
        make_linked_trees(end, getAllDescriptions(), batch_iter_parameters);
    register_handlers(back_handlers);
    for (std::size_t k = 0; k < front_handlers.size(); ++k) {
      extenders.emplace_back(std::make_unique<ExtenderBidirectional>(
          std::move(front_handlers[k]), std::move(back_handlers[k])));
    }
  } break;
  case ExpansionStrategy::Star: {
    auto handlers = make_linked_star_trees(start, getAllDescriptions(),
                                           batch_iter_parameters);
    register_handlers(handlers);
    for (auto &handler : handlers) {
      extenders.emplace_back(
          std::make_unique<ExtenderSingle>(std::move(handler), end));
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
      iter += extender.search();
      if (!extender.getSolutions().empty()) {
        search_predicate.one_solution_was_found = true;
      }
#pragma omp barrier
      for (std::size_t h = 0; h < registered_handlers.size();
           h += omp_get_num_threads()) {
        registered_handlers[h]->drainCaches();
      }
#pragma omp barrier
    }
  });

  recipient.iterations = iter;
  emplace_solution(recipient, get_best_solution(extenders));
  emplace_trees(recipient, extenders);
}
} // namespace mt_rrt
