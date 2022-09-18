/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT-carpet/Error.h>
#include <MT-RRT-core/Planner.h>

#ifdef SHOW_PLANNER_PROGRESS
#include <iostream>
#endif

namespace mt_rrt {
#ifdef SHOW_PLANNER_PROGRESS
void PlannerProgress::reset() {
  std::scoped_lock lock(progress_show_mtx);
  progress_count = 0;
}

PlannerProgress &PlannerProgress::operator++() {
  std::scoped_lock lock(progress_show_mtx);
  ++progress_count;
  std::cout << "iteration: " << progress_count << std::endl;
  return *this;
}
#endif

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

PlannerSolution Planner::solve(const State &start, const State &end,
                               const Parameters &parameters) {
  if (start.size() != state_space_size) {
    throw Error{"size of start is not ", std::to_string(state_space_size)};
  }
  if (end.size() != state_space_size) {
    throw Error{"size of end is not ", std::to_string(state_space_size)};
  }
  std::scoped_lock lock(compute_solution_mtx);
#ifdef SHOW_PLANNER_PROGRESS
  PLANNER_PROGRESS_SINGLETON.reset();
#endif
  auto tic = std::chrono::high_resolution_clock::now();
  PlannerSolution result;
  solve_(start, end, parameters, result);
  result.time = std::chrono::duration_cast<PlannerSolution::TimeUnit>(
      std::chrono::high_resolution_clock::now() - tic);
  return result;
}
} // namespace mt_rrt