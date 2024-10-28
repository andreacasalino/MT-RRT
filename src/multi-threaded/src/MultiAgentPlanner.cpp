/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/MultiAgentPlanner.h>
#include <MT-RRT/TreeUtils.h>
#include <MT-RRT/Types.h>
#include <MT-RRT/extender/Extender.h>

#include "MultiThreadedUtils.h"

#include <algorithm>
#include <future>
#include <optional>
#include <thread>
#include <unordered_map>

namespace mt_rrt {
namespace {
template <bool SimpleOrStar> class ExplorerBase {};

template <> class ExplorerBase<false> {
public:
  auto &getRewires() { return possible_rewires; }

protected:
  std::vector<std::pair<Node *, Rewires>> possible_rewires;
};

template <bool SimpleOrStar>
class Explorer : public extender::ExtenderBase<ExtenderSingle<TreeHandler>>,
                 public ExplorerBase<SimpleOrStar> {
public:
  Explorer(const Node &root, const ProblemDescriptionPtr &problem,
           const Parameters &parameters, const std::vector<float> &target,
           const std::vector<Node *> &master)
      : extender::ExtenderBase<
            ExtenderSingle<TreeHandler>>{make_tree<TreeHandler>(
                                             root.state(), problem, parameters),
                                         target},
        originalRoot{&root}, masterTree{master} {}

  std::size_t search() {
    auto res = extender::ExtenderBase<ExtenderSingle<TreeHandler>>::search();
    if constexpr (!SimpleOrStar) {
      // compure rewires
      DescriptionAndParameters descPars{problem(), this->parameters()};
      for (auto node_it = this->tree_handler->nodes.begin() + 1;
           node_it != this->tree_handler->nodes.end(); ++node_it) {
        NearSet nearSet;
        nearSet.cost2RootSubject = (*node_it)->cost2Root();
        std::size_t tree_size =
            static_cast<std::size_t>(node_it -
                                     this->tree_handler->nodes.begin()) +
            masterTree.size() - 1;
        nearSet.set =
            near_set((*node_it)->state(), this->tree_handler->nodes.begin(),
                     node_it, tree_size, descPars);
        auto nearSet_from_master =
            near_set((*node_it)->state(), masterTree.begin(), masterTree.end(),
                     tree_size, descPars);
        nearSet.set.insert(nearSet.set.end(), nearSet_from_master.begin(),
                           nearSet_from_master.end());
        auto rewires = compute_rewires(**node_it, std::move(nearSet), descPars);
        if (!rewires.involved_nodes.empty()) {
          this->possible_rewires.emplace_back(
              std::make_pair(*node_it, std::move(rewires)));
        }
      }
    }
    return res;
  }

  const Node *originalRoot;

private:
  const std::vector<Node *> &masterTree;
};

template <bool SimpleOrStar> class MasterTreeHandler : public TreeHandler {
public:
  MasterTreeHandler(const View &root, const View &target,
                    const std::vector<ProblemDescriptionPtr> &problems,
                    const Parameters &parameters)
      : TreeHandler(root, problems.front(), parameters),
        target(target.convert()),
        root_sampler(0, 1.f, problems.front()->sampler->sampleSeed()),
        problems{problems} {
    this->parameters.expansion_strategy = ExpansionStrategy::Single;
    explorers.resize(problems.size());
  }

  using ExplorerT = Explorer<SimpleOrStar>;

  ExplorerT &regenerateExplorer(std::size_t th_id) {
    auto &explorer = explorers[th_id];
    auto &problem = problems[th_id];
    std::size_t sampled_root = static_cast<std::size_t>(
        std::floor(root_sampler.sample() * nodes.size()));
    Node *root = nodes[sampled_root];
    return explorer.emplace(*root, problem, this->parameters, this->target,
                            this->nodes);
  }

  void gatherResults() {
    for (auto &explorer : explorers) {
      TreeHandler &hndlr = *explorer->tree_handler;
      // <node in the explorer, counterpart in the master tree>
      std::unordered_map<Node *, Node *> nodes_map;
      auto locateInMasterTree = [&nodes_map](const Node *toFind) {
        auto it = nodes_map.find(const_cast<Node *>(toFind));
        if constexpr (!SimpleOrStar) {
          if (it == nodes_map.end()) {
            // current parent is in master tree and not in the explorer nodes
            // cause compute_rewires found it was better
            // therefore toFind is already a node in the master tree!!
            return const_cast<Node *>(toFind);
          }
        }
        return it->second;
      };
      nodes_map.emplace(hndlr.nodes.front(),
                        const_cast<Node *>(explorer->originalRoot));
      // internalize all explored nodes
      std::for_each(
          hndlr.nodes.begin() + 1, hndlr.nodes.end(), [&](Node *node) {
            auto *added = &this->allocator.emplace_back(node->state());
            nodes_map.emplace(node, added);
            Node *parentInMaster = locateInMasterTree(node->getParent());
            added->setParent(*parentInMaster, node->cost2Go());
            nodes.push_back(added);
          });
      // execute pending rewires
      if constexpr (!SimpleOrStar) {
        for (auto &[parent, rewires] : explorer->getRewires()) {
          for (auto &[involved, _] : rewires.involved_nodes) {
            involved = locateInMasterTree(involved);
          }
          Node *parentInMaster = locateInMasterTree(parent);
          apply_rewires_if_better(*parentInMaster, rewires);
        }
      }
      // internalize the found solutions
      for (const auto &sol : explorer->solutions) {
        auto &added = solutions.emplace_back(sol);
        added.byPassNode = locateInMasterTree(added.byPassNode);
        added.target = View{this->target};
      }
    }
  }

  Solutions<extender::SimpleSolution> solutions;

private:
  std::vector<float> target;
  UniformEngine root_sampler;
  std::vector<ProblemDescriptionPtr> problems;

  std::vector<std::optional<ExplorerT>> explorers;
};

template <bool SimpleOrStar> class Slave {
public:
  static std::vector<std::unique_ptr<Slave>> make(std::size_t threads) {
    std::vector<std::unique_ptr<Slave>> res;
    for (std::size_t k = 0; k < threads; ++k) {
      res.emplace_back(std::make_unique<Slave>());
    }
    return res;
  }

  Slave() : loop_{std::bind(&Slave::run, std::ref(*this))} {}

  ~Slave() {
    active_.store(false, std::memory_order::memory_order_acquire);
    loop_.join();
  }

  std::future<std::size_t> newExploration(Explorer<SimpleOrStar> &extender) {
    auto res = notification_.emplace().get_future();
    explorer_.store(&extender, std::memory_order::memory_order_release);
    return res;
  }

private:
  void run() {
    while (active_.load(std::memory_order::memory_order_acquire)) {
      auto *explr = explorer_.load(std::memory_order::memory_order_acquire);
      if (!explr) {
        continue;
      }
      auto iters = explr->search();
      explorer_.store(nullptr, std::memory_order::memory_order_release);
      notification_->set_value(iters);
    }
  }

  std::thread loop_;
  std::atomic_bool active_{true};
  std::atomic<Explorer<SimpleOrStar> *> explorer_{nullptr};
  std::optional<std::promise<std::size_t>> notification_;
};
} // namespace

void MultiAgentPlanner::solve_(const std::vector<float> &start,
                               const std::vector<float> &end,
                               const Parameters &parameters,
                               PlannerSolution &recipient) {
  auto batched_iterations = compute_batched_iterations(
      parameters.iterations, getThreads(), synchronization());

  auto batch_iter_parameters = parameters;
  batch_iter_parameters.iterations.set(batched_iterations);

  resizeDescriptions(getThreads());

  auto perform = [&](auto &tree, auto &&slaves) {
    std::size_t iter = 0;
    extender::KeepSearchPredicate search_predicate{
        parameters.best_effort, parameters.iterations.get(),
        parameters.expansion_strategy};

    while (search_predicate(iter)) {
      std::vector<std::future<std::size_t>> notifications;
      for (std::size_t k = 0; k < slaves.size(); ++k) {
        auto &extender = tree.regenerateExplorer(k + 1);
        notifications.emplace_back(slaves[k]->newExploration(extender));
      }
      auto &extender = tree.regenerateExplorer(0);
      iter += extender.search();
      // wait for slaves to complete
      for (auto &notification : notifications) {
        notification.wait();
        iter += notification.get();
      }
      tree.gatherResults();
      search_predicate.one_solution_was_found.store(
          !tree.solutions.empty(), std::memory_order::memory_order_release);
    }

    recipient.iterations = iter;
    recipient.solution = materialize_best(tree.solutions);
    recipient.trees.emplace_back(serialize_tree(tree.nodes));
  };

  switch (parameters.expansion_strategy) {
  case ExpansionStrategy::Single: {
    MasterTreeHandler<true> tree{View{start}, View{end}, getAllDescriptions(),
                                 batch_iter_parameters};
    perform(tree, Slave<true>::make(getThreads() - 1));
  } break;
  case ExpansionStrategy::Star: {
    MasterTreeHandler<false> tree{View{start}, View{end}, getAllDescriptions(),
                                  batch_iter_parameters};
    perform(tree, Slave<false>::make(getThreads() - 1));
  } break;
  case ExpansionStrategy::Bidir:
    throw Error{"ExpansionStrategy::Bidir not supported for multi agent"};
    break;
  }
}
} // namespace mt_rrt
