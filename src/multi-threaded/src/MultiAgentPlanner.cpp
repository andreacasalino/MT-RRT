/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/ExtenderUtils.h>
#include <MT-RRT/MultiAgentPlanner.h>

#include "MultiThreadedUtils.h"

#include <algorithm>
#include <unordered_map>

namespace mt_rrt {
namespace {
class MasterTreeHandler : public TreeHandlerBasic {
public:
  MasterTreeHandler(const View &root, const View &target,
                    const std::vector<ProblemDescriptionPtr> &problems,
                    const Parameters &parameters)
      : TreeHandlerBasic(root, problems.front(), parameters),
        target(target.convert()),
        root_sampler(0, 1.f, problems.front()->sampler->sampleSeed()),
        problems{problems} {
    infos.resize(problems.size());
  }

  ExtenderSingle &regenerateExtender() {
    auto &info = infos[omp_get_thread_num()];
    auto &problem = problems[omp_get_thread_num()];
    std::size_t sampled_root = static_cast<std::size_t>(
        std::floor(root_sampler.sample() * nodes.size()));
    Node *root = nodes[sampled_root];
    TreeHandlerPtr hndlr =
        std::make_unique<TreeHandlerBasic>(*root, problem, this->parameters);
    hndlr->parameters.expansion_strategy = ExpansionStrategy::Single;
    return info.emplace(std::move(hndlr), this->target).extender;
  }

  void gatherResults() {
    int th_id = omp_get_thread_num();
    TreeHandlerBasic &hndlr =
        static_cast<TreeHandlerBasic &>(*infos[th_id]->extender.tree_handler);
    hndlr.nodes.erase(hndlr.nodes.begin());

    switch (parameters.expansion_strategy) {
    case ExpansionStrategy::Single:
      break;
    case ExpansionStrategy::Star: {
      computeRewires();
    } break;
    default:
      throw Error{"Invalid gather"};
    }
    if (0 != th_id) {
      return;
    }

    for (auto &info : infos) {
      // internalize all explored nodes
      TreeHandlerBasic &hndlr =
          static_cast<TreeHandlerBasic &>(*info->extender.tree_handler);
      for (Node *node : hndlr.nodes) {
        auto *added = &this->allocator.emplace_back(node->state());
        info->nodes_map.emplace(node, added);
        added->setParent(
            *info->locateInMasterTree(const_cast<Node *>(node->getParent())),
            node->cost2Go());
        nodes.push_back(added);
      }
      // execute pending rewires
      for (auto &[parent, rewires] : info->pending_rewires) {
        for (auto &rew : rewires) {
          rew.involved_node = info->locateInMasterTree(rew.involved_node);
        }
        apply_rewires_if_better(
            *info->locateInMasterTree(const_cast<Node *>(parent)), rewires);
      }
      // gather the found solutions
      for (const auto &sol : info->extender.getSolutions()) {
        auto sol_ptr = std::dynamic_pointer_cast<SimpleSolution>(sol);
        solutions.emplace_back(std::make_shared<SimpleSolution>(
            info->locateInMasterTree(const_cast<Node *>(sol_ptr->byPassNode)),
            sol_ptr->cost2Target, target));
      }
      // reset
      info.reset();
    }
  }

  Solutions solutions;

private:
  std::vector<float> target;
  UniformEngine root_sampler;

  struct ExtendInfo {
    template <typename... Args>
    ExtendInfo(Args &&...extender_args)
        : extender{std::forward<Args>(extender_args)...} {}

    Node *locateInMasterTree(Node *subject) const {
      auto it = nodes_map.find(subject);
      if (it == nodes_map.end()) {
        // this is a node in master tree
        return subject;
      }
      return it->second;
    }

    ExtenderSingle extender;
    std::unordered_map<Node *, Node *>
        nodes_map; // <node in this extender, counterpart in parent tree>
    std::unordered_map<const Node *, std::vector<Rewire>> pending_rewires;
  };
  std::vector<ProblemDescriptionPtr> problems;
  std::vector<std::optional<ExtendInfo>> infos;

  void computeRewires() {
    auto &info = infos[omp_get_thread_num()];
    auto *handler_tree = info->extender.tree_handler.get();
    DescriptionAndParameters descPars{*problems[omp_get_thread_num()],
                                      this->parameters};
    for (auto node_it = handler_tree->nodes.begin();
         node_it != handler_tree->nodes.end(); ++node_it) {
      NearSet nearSet;
      nearSet.cost2RootSubject = (*node_it)->cost2Root();
      nearSet.set = near_set(
          (*node_it)->state(), handler_tree->nodes.begin(), node_it,
          static_cast<std::size_t>(node_it - handler_tree->nodes.begin()),
          descPars);
      auto nearSet_from_master = near_set((*node_it)->state(), nodes.begin(),
                                          nodes.end(), nodes.size(), descPars);
      nearSet.set.insert(nearSet.set.end(), nearSet_from_master.begin(),
                         nearSet_from_master.end());
      for (const auto &rew :
           compute_rewires(**node_it, std::move(nearSet), descPars)) {
        info->pending_rewires[*node_it].push_back(rew);
      }
    }
#pragma omp barrier
  };
};
} // namespace

void MultiAgentPlanner::solve_(const std::vector<float> &start,
                               const std::vector<float> &end,
                               const Parameters &parameters,
                               PlannerSolution &recipient) {
  if (ExpansionStrategy::Bidir == parameters.expansion_strategy) {
    throw Error{"ExpansionStrategy::Bidir not supported for multi agent"};
  }

  auto batched_iterations = compute_batched_iterations(
      parameters.iterations, getThreads(), synchronization());

  auto batch_iter_parameters = parameters;
  batch_iter_parameters.iterations.set(batched_iterations);

  resizeDescriptions(getThreads());
  std::unique_ptr<MasterTreeHandler> master_handler =
      std::make_unique<MasterTreeHandler>(
          View{start}, View{end}, getAllDescriptions(), batch_iter_parameters);

  std::atomic<std::size_t> iter = 0;
  KeepSearchPredicate search_predicate{parameters.best_effort,
                                       parameters.iterations.get(),
                                       parameters.expansion_strategy};

  std::atomic_bool life = true;
  parallel_region(getThreads(), [&]() {
    if (0 == omp_get_thread_num()) {
      // master
      while (search_predicate(iter)) {
#pragma omp barrier
        auto &extender = master_handler->regenerateExtender();
        iter += extender.search();
#pragma omp barrier
        master_handler->gatherResults();
        if (!master_handler->solutions.empty()) {
          search_predicate.one_solution_was_found = true;
        }
      }
      life.store(false);
#pragma omp barrier
    } else {
      // slave
      while (true) {
#pragma omp barrier
        if (!life.load()) {
          break;
        }
        auto &extender = master_handler->regenerateExtender();
        iter += extender.search();
#pragma omp barrier
        master_handler->gatherResults();
      }
    }
  });

  recipient.iterations = iter;
  recipient.solution = find_best_solution(master_handler->solutions);
  recipient.trees.emplace_back(std::move(master_handler));
}
} // namespace mt_rrt
