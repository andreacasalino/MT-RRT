/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/Parameters.h>

#include <chrono>
#include <mutex>
#include <optional>

namespace mt_rrt {

struct PlannerSolution {
  using TimeUnit = std::chrono::milliseconds;

  /** @brief elapsed time
   */
  TimeUnit time;
  /** @brief iterations spent
   */
  std::size_t iterations;
  /** @brief The sequence of states forming the solution to the planning
   * problem. Is an empty vector in case a solution was not found
   */
  std::optional<std::vector<State>> solution;
  /** @brief The trees extended and used in order to solve the problem
   */
  std::vector<Tree> trees;
};

#ifdef SHOW_PLANNER_PROGRESS
class PlannerProgress {
public:
  PlannerProgress() = default;

  void reset();
  PlannerProgress &operator++();

private:
  std::mutex progress_show_mtx;
  std::size_t progress_count = 0;
};

static PlannerProgress PLANNER_PROGRESS_SINGLETON = PlannerProgress{};
#endif

class Planner : public ProblemAware {
public:
  Planner(ProblemDescription &&problem);

  // steal the connector and the sampler contained in the passed problem !!
  Planner(std::shared_ptr<ProblemDescription> problem);

  PlannerSolution solve(const State &start, const State &end,
                        const Parameters &parameters);

protected:
  virtual void solve_(const State &start, const State &end,
                      const Parameters &parameters,
                      PlannerSolution &recipient) = 0;

private:
  std::size_t
      state_space_size; // deduced from sampling a state with the samples

  std::mutex compute_solution_mtx;
};
} // namespace mt_rrt