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
/**
 * @brief Groups all the information characterizing a found solution
 */
struct PlannerSolution {
  using TimeUnit = std::chrono::milliseconds;

  /**
   * @brief computation time spent for obtaining the solution, or trying to get
   * one.
   */
  TimeUnit time;
  /**
   * @brief iterations spent for obtaining the solution, or trying to get
   * one.
   */
  std::size_t iterations;
  /**
   * @brief The sequence of states forming the solution to the planning
   * problem. Is empty in case a solution was not found
   */
  std::optional<std::vector<State>> solution;
  /**
   * @brief The search trees produced while trying to solve the problem
   */
  std::vector<Tree> trees;
};

// if you enable this flag, all possible planner will display the iterations in
// the console while running. This could be done mainly for debug purpose and
// you should be aware that of courde affects performances.
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

/**
 * @brief In order to solve any kind of problem you need to build a kind of
 * Planner, passing for sure the @ProblemDescription.
 * The same @Planner, can be used to solve (one at a time) multiple problems,
 * calling many times Planner::solve(...), possibly using different (or not)
 * @Parameters every time.
 */
class Planner : public ProblemAware {
public:
  /**
   * @brief steals the connector and the sampler contained in the passed problem
   */
  Planner(ProblemDescription &&problem);

  /**
   * @brief steals the connector and the sampler contained in the passed problem
   */
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
