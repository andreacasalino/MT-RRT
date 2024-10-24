/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/TreeUtils.h>
#include <MT-RRT/LinkedTreesPlanner.h>
#include <MT-RRT/Channel.h>

#include "MultiThreadedUtils.h"

#include <algorithm>
#include <omp.h>
#include <unordered_map>
#include <variant>
#include <deque>
#include <type_traits>

namespace mt_rrt {
namespace {
struct SteeredNodeMsg {
  Node* involved;
  Node* parent;
  float costFromParent;
};
struct RewiredNodeMsg {
  Node* involved;
  Node* parent;
  float costFromParent;
};
using Msg = std::variant<SteeredNodeMsg, RewiredNodeMsg>;

template<typename T>
class OutChannel : public Channel<T> {
public:
  using Channel<T>::Channel;

  void push(T&& to_add) {
    if(!queue->push(std::forward<T>(to_add))) {
      pending.emplace_back(std::forward<T>(to_add));
    }
  }

  std::deque<Msg> pending;
};
template<typename T>
using OutChannelPtr = std::shared_ptr<OutChannel<T>>;

template<bool SimpleOrStar>
class LinkedTreeHandlerBase {};

template<>
class LinkedTreeHandlerBase<true> {
protected:
  std::vector<ChannelPtr<Node*>> incoming;
  std::vector<OutChannelPtr<Node*>> outgoing;
};

template<>
class LinkedTreeHandlerBase<false> {
protected:
  std::vector<ChannelPtr<Msg>> incoming;
  std::vector<OutChannelPtr<Msg>> outgoing;

  void addToTables(Node* original, Node* inThisNode) {
    original2ThisNodes.emplace(original, inThisNode);
    thisNodes2Original.emplace(inThisNode, original);
  }

  std::unordered_map<const Node*, Node*> original2ThisNodes;
  std::unordered_map<const Node*, Node*> thisNodes2Original;
};

template<bool SimpleOrStar>
class LinkedTreeHandler 
: public TreeHandler
, public LinkedTreeHandlerBase<SimpleOrStar> {
public:
  Node *internalize(const Node &subject) {
    Node *added = this->TreeHandler::internalize(subject);
    this->addToTables(added, addead);
    if constexpr (SimpleOrStar) {
      for(auto& channel : this->outgoing) {
        channel->push(added);
      }
    }
    else {
      SteeredNodeMsg msg{added, this->thisNodes2Original[added->getParent()], added->cost2Go()};
      for(auto& channel : this->outgoing) {
        channel->push(msg);
      }
    }
    return added;
  }

  void applyRewires(const Node &parent, const Rewires &rewires) {
    this->TreeHandler::applyRewires(parent, rewires);
    if constexpr(SimpleOrStar) {
      for(auto [node, cost] : rewires.involved_nodes) {
        for(auto& channel : this->outgoing) {
          channel->push(RewiredNodeMsg{thisNodes2Original[node], thisNodes2Original[parent], cost});
        }
      }
    }
  }

  /////////////////////////////////////////////////////////////////////////////

  static std::vector<std::unique_ptr<LinkedTreeHandler>>
  make_trees(const View &root,
             const std::vector<ProblemDescriptionPtr> &problems,
             const Parameters &parameters) {
    std::vector<std::unique_ptr<LinkedTreeHandler>> res;
    std::vector<Node *> roots;
    for (std::size_t th_id = 0; th_id < problems.size(); ++th_id) {
      res.emplace_back().reset(
          new LinkedTreeHandler(root, problems[th_id], parameters, th_id));
      roots.push_back(res.back()->nodes.front());
    }
    auto allTrees = std::make_shared<std::vector<LinkedTreeHandler *>>();
    for (const auto &tree : res) {
      allTrees->push_back(tree.get());
    }
    auto sharedRegister = std::make_shared<GlobalRegister>(roots);
    for (auto &tree : res) {
      tree->sharedRegister = sharedRegister;
      tree->allTrees = allTrees;
    }
    return res;
  }

  void pollChannels() {
    for(auto& channel : this->incoming) {
      if constexpr (SimpleOrStar) {
        Node* n;
        for(std::uint8_t k =0; k<MAX_SUCCESS_POLL && channel.poll(n); ++k) {
          this->nodes.push_back(n);
        }
      }

      else {
        Msg msg;
        for(std::uint8_t k =0; k<MAX_SUCCESS_POLL && channel.poll(msg); ++k) {
          std::visit(msg, [](auto msg){
            Node* involved, * parent;
            if(auto it = this->original2ThisNodes.find(msg.parent); it != this->original2ThisNodes.end()) {
              parent = it->second;
            }
            else {
              return;
            }
            if constexpr (std::is_same_v<decltype(msg), SteeredNodeMsg>) {
              auto &added = this->allocator.emplace_back(involved->state());
              added.setParent(*parent, msg.costFromParent);
              nodes.push_back(&added);
              this->addToTables(involved, &addead);
            }
            if constexpr (std::is_same_v<decltype(msg), RewiredNodeMsg>) {
              if(auto it = this->original2ThisNodes.find(involved); it != this->original2ThisNodes.end()) {
                involved = it->second;
              }
              else {
                return
              involved->setParent(*parent, msg.costFromParent);
            }
          });
        }
      }

    }
  }

private:
  static const inline std::uint8_t MAX_SUCCESS_POLL = 3;

  LinkedTreeHandler(const View &root, const ProblemDescriptionPtr &problem,
                    const Parameters &parameters, std::size_t th_id)
      : TreeHandler(root, problem, parameters) {}

  Channels<SimpleOrStar> channels_;
  std::unordered_map<Node*, Node*> nodesTable_; // < original node, copy in this tree (or itself if this is the originating tree) >
};
} // namespace

void LinkedTreesPlanner::solve_(const std::vector<float> &start,
                                const std::vector<float> &end,
                                const Parameters &parameters,
                                PlannerSolution &recipient) {
  resizeDescriptions(getThreads());
  Extenders extenders;
  std::vector<LinkedTreeHandler *> handlers;

  auto batched_iterations = compute_batched_iterations(
      parameters.iterations, getThreads(), synchronization());

  auto batch_iter_parameters = parameters;
  batch_iter_parameters.iterations.set(batched_iterations);

  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single:
  case ExpansionStrategy::Star: {
    for (auto &&tree : LinkedTreeHandler::make_trees(
             start, getAllDescriptions(), batch_iter_parameters)) {
      handlers.push_back(tree.get());
      extenders.emplace_back(
          std::make_unique<ExtenderSingle>(std::move(tree), end));
    }
  } break;
  case ExpansionStrategy::Bidir: {
    auto front_trees = LinkedTreeHandler::make_trees(
        start, getAllDescriptions(), batch_iter_parameters);
    for (const auto &tree : front_trees) {
      handlers.push_back(tree.get());
    }
    auto back_trees = LinkedTreeHandler::make_trees(end, getAllDescriptions(),
                                                    batch_iter_parameters);
    for (const auto &tree : back_trees) {
      handlers.push_back(tree.get());
    }
    for (std::size_t k = 0; k < front_trees.size(); ++k) {
      extenders.emplace_back(std::make_unique<ExtenderBidirectional>(
          std::move(front_trees[k]), std::move(back_trees[k])));
    }
  } break;
  }

  std::atomic<std::size_t> iter = 0;
  KeepSearchPredicate search_predicate{parameters.best_effort,
                                       parameters.iterations.get(),
                                       parameters.expansion_strategy};

  parallel_region(getThreads(), [&]() {
    Extender &extender = *extenders[omp_get_thread_num()];
    while (search_predicate(iter)) {
      for_each_handler(handlers, [](LinkedTreeHandler &hndlr) {
        hndlr.resetExtensionCache();
      });
      iter += extender.search();
      if (!extender.getSolutions().empty()) {
        search_predicate.one_solution_was_found = true;
      }
#pragma omp barrier
      for_each_handler(handlers, [](LinkedTreeHandler &hndlr) {
        hndlr.internalizeResults();
      });
#pragma omp barrier
      if (omp_get_thread_num() != 0)
        continue;
      for_each_handler(handlers, [](LinkedTreeHandler &hndlr) {
        hndlr.updateGlobalRegister();
      });
    }
  });

  recipient.iterations = iter;
  recipient.solution = get_best_solution(extenders);
  emplace_trees(recipient, extenders);
}
} // namespace mt_rrt
