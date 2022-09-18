/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT-multi-threaded/MultiAgentPlanner.h>

#include <MultiThreadedUtils.h>

#include <algorithm>

namespace mt_rrt {
namespace {
using RewiresMap = std::unordered_multimap<Node *, Rewires>;

template <typename Container>
typename Container::const_iterator at_second(const Container &subject) {
  auto it = subject.begin();
  ++it;
  return it;
}

class MasterTreeHandler : public TreeHandler {
public:
  MasterTreeHandler(
      const State &root, const State &target,
      const std::vector<ProblemDescriptionPtr> &problems,
      const Parameters &parameters,
      MultiAgentPlanner::StarExpansionStrategyApproach star_approach)
      : TreeHandler(make_root(root), problems.front(), parameters),
        target(target),
        root_sampler(0, 1.f, problems.front()->sampler->sampleSeed()),
        star_approach(star_approach) {
    for (const auto &problem : problems) {
      auto &params = infos.emplace_back(ExtendInfo{problem, parameters}).params;
      params.expansion_strategy = ExpansionStrategy::Single;
    }
  }

  Extender &getExtender() {
    TreePtr extender_tree;
    extender_tree = std::make_shared<Tree>();
    {
      NodePtr root;
      {
        std::scoped_lock lock(root_sampler_mtx);
        auto pos = static_cast<std::size_t>(
            std::floor(root_sampler.sample() * tree->size()));
        auto it = tree->begin();
        std::advance(it, pos);
        root = *it;
      }
      extender_tree->push_back(root);
    }

    auto &info = infos[omp_get_thread_num()];
    info.extender = std::make_unique<ExtenderSingle>(
        std::make_unique<TreeHandler>(extender_tree, info.problem, info.params),
        target);
    info.nodes_to_collect.clear();
    info.rewires_to_apply.clear();
    return *info.extender;
  }

  void gatherResults() {
    switch (parameters.expansion_strategy) {
    case ExpansionStrategy::Single:
      normalGather();
      return;
    case ExpansionStrategy::Star: {
      switch (star_approach) {
      case MultiAgentPlanner::StarExpansionStrategyApproach::ExploitAllThreads:
        multiThreadedStarGather();
        break;
      case MultiAgentPlanner::StarExpansionStrategyApproach::MonoThread:
        monoThreadStarGather();
        break;
      default:
        throw Error{"Invalid gather"};
        break;
      }
    }
      return;
    default:
      break;
    }
    throw Error{"Invalid gather"};
  }

  Solutions solutions;

private:
  const MultiAgentPlanner::StarExpansionStrategyApproach star_approach;
  const State &target;

  std::mutex root_sampler_mtx;
  UniformEngine root_sampler;

  struct ExtendInfo {
    ProblemDescriptionPtr problem;
    Parameters params;
    std::unique_ptr<ExtenderSingle> extender;
    Tree nodes_to_collect;
    RewiresMap rewires_to_apply;
  };
  std::vector<ExtendInfo> infos;

  void normalGather() {
    if (0 != omp_get_thread_num()) {
      return;
    }
    for (const auto &info : infos) {
      auto &giver = *info.extender->tree_handler->tree;
      std::for_each(
          at_second(giver), giver.cend(),
          [this](const NodePtr &node) { this->tree->push_back(node); });
      for (const auto &[cost, solution] : info.extender->getSolutions()) {
        this->solutions.emplace(cost, solution);
      }
    }
  }

  void multiThreadedStarGather() {
    const auto th_id = omp_get_thread_num();
    {
      auto &info = infos[th_id];
      auto *handler_tree = info.extender->tree_handler.get();
      handler_tree->tree->pop_front();
      info.nodes_to_collect = *handler_tree->tree;
      *handler_tree->tree = *this->tree;
      const auto &problem = *info.problem;
      std::unordered_set<Node *> sorted_nodes_to_collect;
      for (const auto &n : info.nodes_to_collect) {
        sorted_nodes_to_collect.emplace(n.get());
      }
      for (const auto &found_node : info.nodes_to_collect) {
        auto near_set = handler_tree->nearSet(*found_node);
        auto &rewires_to_add =
            info.rewires_to_apply.emplace(found_node.get(), Rewires{})->second;
        for (const auto &rewire : compute_rewires(
                 *found_node, near_set,
                 DescriptionAndParameters{problem, this->parameters})) {
          if (sorted_nodes_to_collect.find(&rewire.involved_node) ==
              sorted_nodes_to_collect.end()) {
            // this rewire involves a node in the common tree: do it later from
            // main thread
            rewires_to_add.push_back(rewire);
          } else {
            // this rewire involves a node in the found nodes list: it can be
            // applied now
            rewire.involved_node.setFatherInfo(
                NodeFatherInfo{found_node.get(), rewire.new_cost_from_father});
          }
        }
        handler_tree->tree->push_back(found_node);
      }
    }
#pragma omp barrier
    if (0 != th_id) {
      return;
    }
    for (const auto &info : infos) {
      this->tree->insert(this->tree->end(), info.nodes_to_collect.begin(),
                         info.nodes_to_collect.end());
      for (auto &found_rewire : info.rewires_to_apply) {
        Node &new_father = *found_rewire.first;
        this->applyRewires(new_father, found_rewire.second);
      }
    }
    foundSolutionsGather();
  }

  void monoThreadStarGather() {
    if (0 != omp_get_thread_num()) {
      return;
    }
    for (auto &info : infos) {
      auto &info_tree = *info.extender->tree_handler->tree;
      std::for_each(
          at_second(info_tree), info_tree.cend(), [this](const NodePtr &node) {
            const auto near_set = this->nearSet(*node);
            auto rewires = compute_rewires(
                *node, near_set,
                DescriptionAndParameters{this->problem(), this->parameters});
            this->tree->push_back(node);
            this->applyRewires(*node, rewires);
          });
    }
    foundSolutionsGather();
  }

  void foundSolutionsGather() {
    for (const auto &info : infos) {
      for (const auto &[cost, solution] : info.extender->getSolutions()) {
        const auto *casted_solution =
            dynamic_cast<const SingleSolution *>(solution.get());
        const auto solution_updated_cost =
            casted_solution->by_pass_node.cost2Root() +
            info.problem->connector->minCost2Go(
                casted_solution->by_pass_node.getState(), target);
        this->solutions.emplace(solution_updated_cost, solution);
      }
    }
  }
};
} // namespace

void MultiAgentPlanner::solve_(const State &start, const State &end,
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
  MasterTreeHandler master_handler(start, end, getAllDescriptions(),
                                   batch_iter_parameters, star_approach);

  std::atomic<std::size_t> iter = 0;
  KeepSearchPredicate search_predicate{parameters.best_effort,
                                       parameters.iterations.get(),
                                       parameters.expansion_strategy};

  std::atomic_bool stop = false;
  parallel_region(getThreads(), [&]() {
    if (0 == omp_get_thread_num()) {
      while (search_predicate(iter)) {
#pragma omp barrier
        auto &extender = master_handler.getExtender();
        iter += extender.search();
#pragma omp barrier
        master_handler.gatherResults();
        if (!master_handler.solutions.empty()) {
          search_predicate.one_solution_was_found = true;
        }
      }
      stop = true;
#pragma omp barrier
    } else {
      while (true) {
#pragma omp barrier
        if (stop) {
          break;
        }
        auto &extender = master_handler.getExtender();
        iter += extender.search();
#pragma omp barrier
        master_handler.gatherResults();
      }
    }
  });

  recipient.iterations = iter;
  emplace_solution(recipient, master_handler.solutions);
  recipient.trees = {*master_handler.tree};
}
} // namespace mt_rrt
