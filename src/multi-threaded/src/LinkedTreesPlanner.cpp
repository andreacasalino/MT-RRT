/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Channel.h>
#include <MT-RRT/LinkedTreesPlanner.h>
#include <MT-RRT/TreeUtils.h>

#include "MultiThreadedUtils.h"

#include <algorithm>
#include <deque>
#include <type_traits>
#include <unordered_map>

#ifdef SHOW_PLANNER_PROGRESS
#include <MT-RRT/Progress.h>
#endif

namespace mt_rrt {
namespace {
struct Msg {
  enum class Kind { SteeredNode, Rewire };

  Kind kind;
  Node *involved;
  Node *parent;
  float costFromParent;
};

template <bool SimpleOrStar> class LinkedTreeHandlerBase {};

template <> class LinkedTreeHandlerBase<true> : public Network<Node *> {
public:
  using NetworkT = Network<Node *>;
};

template <> class LinkedTreeHandlerBase<false> : public Network<Msg> {
public:
  using NetworkT = Network<Msg>;

protected:
  void addToTables(Node *original, Node *inThisNode) {
    original2ThisNodes.emplace(original, inThisNode);
    thisNodes2Original.emplace(inThisNode, original);
  }

  void addToTablesRoots(const std::vector<Node *> &roots, std::size_t th_id) {
    Node *my_root = roots[th_id];
    for (auto *root : roots) {
      original2ThisNodes.emplace(root, my_root);
    }
    thisNodes2Original.emplace(my_root, my_root);
  }

  Node *findFromOriginal(const Node *original) {
    return original2ThisNodes.find(original)->second;
  }

  Node *findFromThisNode(const Node *thisNode) {
    return thisNodes2Original.find(thisNode)->second;
  }

private:
  std::unordered_map<const Node *, Node *> original2ThisNodes;
  std::unordered_map<const Node *, Node *> thisNodes2Original;
};

template <bool SimpleOrStar>
class LinkedTreeHandler : public TreeHandler,
                          public LinkedTreeHandlerBase<SimpleOrStar> {
public:
  Node *internalize(const Node &subject) {
    Node *added = this->TreeHandler::internalize(subject);
    if constexpr (SimpleOrStar) {
      this->push(added);
    } else {
      this->addToTables(added, added);
      this->push(Msg{Msg::Kind::SteeredNode, added,
                     this->findFromThisNode(added->getParent()),
                     added->cost2Go()});
    }
    return added;
  }

  void applyRewires(const Node &parent, const Rewires &rewires) {
    this->TreeHandler::applyRewires(parent, rewires);
    if constexpr (!SimpleOrStar) {
      for (auto [node, cost] : rewires.involved_nodes) {
        this->push(Msg{Msg::Kind::Rewire, this->findFromThisNode(node),
                       const_cast<Node *>(&parent), cost});
      }
    }
  }

  /////////////////////////////////////////////////////////////////////////////

  static std::vector<TreeHandlerPtr<LinkedTreeHandler>>
  make_trees(const View &root,
             const std::vector<ProblemDescriptionPtr> &problems,
             const Parameters &parameters) {
    std::vector<TreeHandlerPtr<LinkedTreeHandler>> res;
    for (auto &problem : problems) {
      res.emplace_back().reset(
          new LinkedTreeHandler{root, problem, parameters});
    }
    using NetworkT = typename LinkedTreeHandlerBase<SimpleOrStar>::NetworkT;
    std::vector<std::reference_wrapper<NetworkT>> refs;
    for (const auto &ptr : res) {
      refs.emplace_back(*ptr.get());
    }
    NetworkT::setUp(refs, 15);

    if constexpr (!SimpleOrStar) {
      std::vector<Node *> roots;
      for (const auto &tree : res) {
        roots.push_back(tree->nodes.front());
      }
      for (std::size_t th_id = 0; th_id < res.size(); ++th_id) {
        res[th_id]->addToTablesRoots(roots, th_id);
        // res[th_id]->print();
      }
    }
    return res;
  }

  void pollChannels() {
    if constexpr (SimpleOrStar) {
      this->poll(MAX_SUCCESS_POLL,
                 [this](Node *n) { this->nodes.push_back(n); });
    }

    else {
      this->poll(MAX_SUCCESS_POLL, [this](const Msg &msg) {
        Node *parent = this->findFromOriginal(msg.parent);
        switch (msg.kind) {
        case Msg::Kind::SteeredNode: {
          auto &added = this->allocator.emplace_back(msg.involved->state());
          added.setParent(*parent, msg.costFromParent);
          nodes.push_back(&added);
          this->addToTables(msg.involved, &added);
        } break;

        case Msg::Kind::Rewire: {
          Node *involved = this->findFromOriginal(msg.involved);
          if (msg.costFromParent < involved->cost2Root()) {
            involved->setParent(*parent, msg.costFromParent);
          }
        } break;
        }
      });
    }
  }

private:
  static const inline std::uint8_t MAX_SUCCESS_POLL = 5;

  LinkedTreeHandler(const View &root, const ProblemDescriptionPtr &problem,
                    const Parameters &parameters)
      : TreeHandler(root, problem, parameters) {}
};

template <template <typename> class Implementation, bool SimpleOrStar>
class ExtenderBase_ : public extender::ExtenderBase<
                          Implementation<LinkedTreeHandler<SimpleOrStar>>> {
public:
  using extender::ExtenderBase<
      Implementation<LinkedTreeHandler<SimpleOrStar>>>::ExtenderBase;

  void search(std::atomic<std::size_t> &iter) {
    const auto &pars = this->parameters();
    extender::KeepSearchPredicate search_predicate{
        pars.best_effort, pars.iterations.get(), pars.expansion_strategy};

    for (; search_predicate(iter.load(std::memory_order::memory_order_acquire));
         ++iter) {
      this->search_iteration(
          this->solutions,
          this->determinismManager_.doDeterministicExtension());
      search_predicate.one_solution_was_found.store(
          !this->solutions.empty(), std::memory_order::memory_order_release);

      if constexpr (std::is_same_v<
                        Implementation<LinkedTreeHandler<SimpleOrStar>>,
                        extender::Single<LinkedTreeHandler<true>>> ||
                    std::is_same_v<
                        Implementation<LinkedTreeHandler<SimpleOrStar>>,
                        extender::Single<LinkedTreeHandler<false>>>) {
        this->tree_handler->pollChannels();
      } else {
        this->front_handler->pollChannels();
        this->back_handler->pollChannels();
      }

#ifdef SHOW_PLANNER_PROGRESS
      ++Progress::get();
#endif
    }
  }
};
} // namespace

template <template <typename> class Implementation>
class Extender<Implementation, LinkedTreeHandler<true>>
    : public ExtenderBase_<Implementation, true> {
public:
  using ExtenderBase_<Implementation, true>::ExtenderBase_;
};

template <template <typename> class Implementation>
class Extender<Implementation, LinkedTreeHandler<false>>
    : public ExtenderBase_<Implementation, false> {
public:
  using ExtenderBase_<Implementation, false>::ExtenderBase_;
};

void LinkedTreesPlanner::solve_(const std::vector<float> &start,
                                const std::vector<float> &end,
                                const Parameters &parameters,
                                PlannerSolution &recipient) {
  resizeDescriptions(getThreads());

  auto perform = [&](auto &&extenders) {
    std::atomic<std::size_t> iter = 0;

    parallel_region(getThreads(), [&](std::size_t th_id) {
      auto &extender = extenders[th_id];
      extender.search(iter);
    });

    recipient.iterations = iter;
    recipient.solution = materialize_best_in_extenders(extenders);
    serializeTrees(extenders, parameters, recipient);
  };

  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single: {
    perform(make_single_extenders(LinkedTreeHandler<true>::make_trees(
                                      start, getAllDescriptions(), parameters),
                                  end));
  } break;

  case ExpansionStrategy::Star: {
    perform(make_single_extenders(LinkedTreeHandler<false>::make_trees(
                                      start, getAllDescriptions(), parameters),
                                  end));
  } break;

  case ExpansionStrategy::Bidir: {
    perform(make_bidirectional_extenders(
        LinkedTreeHandler<true>::make_trees(start, getAllDescriptions(),
                                            parameters),
        LinkedTreeHandler<true>::make_trees(end, getAllDescriptions(),
                                            parameters)));
  } break;
  }
}
} // namespace mt_rrt
