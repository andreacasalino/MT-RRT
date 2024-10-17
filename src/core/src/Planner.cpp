/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Error.h>
#include <MT-RRT/Planner.h>

#ifdef SHOW_PLANNER_PROGRESS
#include "Progress.h"
#endif

#include <unordered_map>

namespace mt_rrt {

PlannerSolution::TreeSerialized
serialize_tree(const std::vector<Node *> &nodes) {
  std::unordered_map<const Node *, std::size_t> indicesMap{{nullptr, 0}};
  for (std::size_t index = 0; index < nodes.size(); ++index) {
    indicesMap.emplace(nodes[index], index);
  }
  PlannerSolution::TreeSerialized res;
  for (auto *node : nodes) {
    res.emplace_back(PlannerSolution::NodeSerialized{
        node->cost2Go(), indicesMap[node->getParent()],
        node->state().convert()});
  }
  return res;
}

namespace {
ProblemDescriptionPtr make_description(ProblemDescription &&problem) {
  if (nullptr == problem.sampler) {
    throw Error{"Empty sampler"};
  }
  if (nullptr == problem.connector) {
    throw Error{"Empty connector"};
  }

  std::shared_ptr<ProblemDescription> result =
      std::make_shared<ProblemDescription>();
  result->sampler = std::move(problem.sampler);
  result->connector = std::move(problem.connector);
  result->simmetry = problem.simmetry;
  result->gamma.set(problem.gamma.get());
  return result;
}
} // namespace

Planner::Planner(ProblemDescription &&problem)
    : ProblemAware{
          make_description(std::forward<ProblemDescription>(problem))} {
  state_space_size = this->problem().sampler->sampleState().size();
}

Planner::Planner(std::shared_ptr<ProblemDescription> problem)
    : Planner(std::move(*problem)) {}

PlannerSolution Planner::solve(const std::vector<float> &start,
                               const std::vector<float> &end,
                               const Parameters &parameters) {
  if (start.size() != state_space_size) {
    throw Error{"size of start is not ", std::to_string(state_space_size)};
  }
  if (end.size() != state_space_size) {
    throw Error{"size of end is not ", std::to_string(state_space_size)};
  }
  std::scoped_lock lock(compute_solution_mtx);
#ifdef SHOW_PLANNER_PROGRESS
  Progress::get().reset();
#endif
  auto tic = std::chrono::high_resolution_clock::now();
  PlannerSolution result;
  solve_(start, end, parameters, result);
  result.time = std::chrono::duration_cast<PlannerSolution::TimeUnit>(
      std::chrono::high_resolution_clock::now() - tic);
  return result;
}
} // namespace mt_rrt