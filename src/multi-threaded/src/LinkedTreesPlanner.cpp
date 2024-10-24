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
#include <omp.h>
#include <type_traits>
#include <unordered_map>
#include <variant>

#ifdef SHOW_PLANNER_PROGRESS
#include <MT-RRT/Progress.h>
#endif

namespace mt_rrt {
namespace {
struct SteeredNodeMsg {
  Node *involved;
  Node *parent;
  float costFromParent;
};
struct RewiredNodeMsg {
  Node *involved;
  Node *parent;
  float costFromParent;
};
using Msg = std::variant<SteeredNodeMsg, RewiredNodeMsg>;

template <typename T> class OutChannel : public Channel<T> {
public:
  OutChannel(ChannelPtr<T> chnl) : channel{chnl} {}

  void push(T &&to_add) {
    if (!channel->push(std::forward<T>(to_add))) {
      pending.emplace_back(std::forward<T>(to_add));
    }
  }

  ChannelPtr<T> channel;
  std::deque<T> pending;
};

template <typename T> class Network {
protected:
  template <typename U> static void setUp(std::vector<U> &subject) {
    std::vector<Network<T> *> ptrs;
    for (auto &s : subject) {
      ptrs.push_back(s.get());
    }
    std::vector<std::vector<ChannelPtr<T>>> grid;
    grid.resize(subject.size());
    for (auto &row : grid) {
      for (std::size_t k = 0; k < subject.size(); ++k) {
        row.emplace_back(std::make_shared<Channel<T>>());
      }
    }
    for (std::size_t k = 0; k < subject.size(); ++k) {
      for (std::size_t c = 0; c < subject.size(); ++c) {
        if (c == k) {
          continue;
        }
        auto channel = grid[k][c];
        ptrs[k]->outgoing.emplace_back(channel);
      }
      for (std::size_t r = 0; r < subject.size(); ++r) {
        if (r == k) {
          continue;
        }
        auto channel = grid[r][k];
        ptrs[k]->incoming.emplace_back(channel);
      }
    }
  }

  void push(const T &to_add) {
    for (auto &channel : outgoing) {
      channel->push(to_add);
    }
  }

  std::vector<ChannelPtr<T>> incoming;
  std::vector<OutChannel<T>> outgoing;
};

template <bool SimpleOrStar> class LinkedTreeHandlerBase {};

template <> class LinkedTreeHandlerBase<true> : public Network<Node *> {};

template <> class LinkedTreeHandlerBase<false> : public Network<Msg> {
protected:
  void addToTables(Node *original, Node *inThisNode) {
    original2ThisNodes.emplace(original, inThisNode);
    thisNodes2Original.emplace(inThisNode, original);
  }

  std::unordered_map<const Node *, Node *> original2ThisNodes;
  std::unordered_map<const Node *, Node *> thisNodes2Original;
};

template <bool SimpleOrStar>
class LinkedTreeHandler : public TreeHandler,
                          public LinkedTreeHandlerBase<SimpleOrStar> {
public:
  Node *internalize(const Node &subject) {
    Node *added = this->TreeHandler::internalize(subject);
    this->addToTables(added, added);
    if constexpr (SimpleOrStar) {
      this->push(added);
    } else {
      this->push(SteeredNodeMsg{added,
                                this->thisNodes2Original[added->getParent()],
                                added->cost2Go()});
    }
    return added;
  }

  void applyRewires(const Node &parent, const Rewires &rewires) {
    this->TreeHandler::applyRewires(parent, rewires);
    if constexpr (SimpleOrStar) {
      for (auto [node, cost] : rewires.involved_nodes) {
        this->push(RewiredNodeMsg{node, thisNodes2Original[parent], cost});
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
      res.emplace_back(make_tree<LinkedTreeHandler>(root, problem, parameters));
    }
    setUp(res);
    return res;
  }

  void pollChannels() {
    for (auto &channel : this->incoming) {
      if constexpr (SimpleOrStar) {
        Node *n;
        for (std::uint8_t k = 0; k < MAX_SUCCESS_POLL && channel.poll(n); ++k) {
          this->nodes.push_back(n);
        }
      }

      else {
        Msg msg;
        for (std::uint8_t k = 0; k < MAX_SUCCESS_POLL && channel.poll(msg);
             ++k) {
          std::visit(
              [](auto msg) {
                Node *involved, *parent;
                if (auto it = this->original2ThisNodes.find(msg.parent);
                    it != this->original2ThisNodes.end()) {
                  parent = it->second;
                } else {
                  return;
                }

                if constexpr (std::is_same_v<decltype(msg), SteeredNodeMsg>) {
                  auto &added = this->allocator.emplace_back(involved->state());
                  added.setParent(*parent, msg.costFromParent);
                  nodes.push_back(&added);
                  this->addToTables(involved, &added);
                }

                if constexpr (std::is_same_v<decltype(msg), RewiredNodeMsg>) {
                  if (auto it = this->original2ThisNodes.find(involved);
                      it != this->original2ThisNodes.end()) {
                    involved = it->second;
                  } else {
                    return involved->setParent(*parent, msg.costFromParent);
                  }
                }
              },
              msg);
        }
      }
    }
  }

private:
  static const inline std::uint8_t MAX_SUCCESS_POLL = 3;

  LinkedTreeHandler(const View &root, const ProblemDescriptionPtr &problem,
                    const Parameters &parameters)
      : TreeHandler(root, problem, parameters) {}
};

template <bool SimpleOrStar, template <typename> class ExtenderImpl>
class SharedTreeExtender
    : public extender::Extender<ExtenderImpl<LinkedTreeHandler<SimpleOrStar>>> {
public:
  using extender::Extender<
      ExtenderImpl<LinkedTreeHandler<SimpleOrStar>>>::Extender;

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
                        ExtenderImpl<LinkedTreeHandler<SimpleOrStar>>,
                        extender::Single<LinkedTreeHandler<SimpleOrStar>>>) {
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

void LinkedTreesPlanner::solve_(const std::vector<float> &start,
                                const std::vector<float> &end,
                                const Parameters &parameters,
                                PlannerSolution &recipient) {
  resizeDescriptions(getThreads());

  auto perform = [&](auto &extenders) {
    std::atomic<std::size_t> iter = 0;

    parallel_region(getThreads(), [&]() {
      const auto th_id = omp_get_thread_num();
      auto &extender = extenders[th_id];
      extender.search(iter);
    });

    recipient.iterations = iter;
    recipient.solution = materialize_best_in_extenders(extenders);
    serializeTrees(extenders, parameters, recipient);
  };

  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single: {
    auto trees = LinkedTreeHandler<true>::make_trees(
        start, getAllDescriptions(), parameters);
    std::vector<SharedTreeExtender<true, extender::Single>> extenders;
    for (auto &tree : trees) {
      extenders.emplace_back(std::move(tree), end);
    }
    perform(extenders);
  } break;

  case ExpansionStrategy::Star: {
    auto trees = LinkedTreeHandler<false>::make_trees(
        start, getAllDescriptions(), parameters);
    std::vector<SharedTreeExtender<false, extender::Single>> extenders;
    for (auto &tree : trees) {
      extenders.emplace_back(std::move(tree), end);
    }
    perform(extenders);
  } break;

  case ExpansionStrategy::Bidir: {
    auto front_trees = LinkedTreeHandler<true>::make_trees(
        start, getAllDescriptions(), parameters);
    auto back_trees = LinkedTreeHandler<true>::make_trees(
        end, getAllDescriptions(), parameters);
    std::vector<SharedTreeExtender<false, extender::Bidirectional>> extenders;
    for (std::size_t k = 0; k < front_trees.size(); ++k) {
      extenders.emplace_back(std::move(front_trees[k]),
                             std::move(back_trees[k]));
    }
    perform(extenders);
  } break;
  }
}
} // namespace mt_rrt
