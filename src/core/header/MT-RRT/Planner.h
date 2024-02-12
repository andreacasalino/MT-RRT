/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/ProblemDescription.h>
#include <MT-RRT/TreeHandler.h>

#include <mutex>

namespace mt_rrt {
/**
 * @brief Groups all the information characterizing a found solution
 */
struct PlannerSolution {
  using TimeUnit = std::chrono::microseconds;

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
  std::vector<std::vector<float>> solution;
  /**
   * @brief The search trees produced while trying to solve the problem
   */
  std::vector<TreeHandlerPtr> trees;
};

/**
 * @brief In order to solve any kind of problem you need to build a kind of
 * Planner, passing for sure the @ProblemDescription.
 * The same @Planner, can be used to solve (one at a time) multiple problems,
 * calling many times Planner::solve(...), possibly using different (or not)
 * @Parameters every time.
 *
 * When enabling SHOW_PLANNER_PROGRESS, all instantiated planners will display
 * the iterations in the console while running. This could be done mainly for
 * debug purpose and you should be aware that this of course affects
 * performances.
 */
class Planner : public ProblemAware {
public:
  /**
   * @brief steals the connector and the sampler contained in the passed problem
   */
  Planner(ProblemDescription &&problem);

  PlannerSolution solve(const std::vector<float> &start,
                        const std::vector<float> &end,
                        const Parameters &parameters);

protected:
  /**
   * @brief steals the connector and the sampler contained in the passed
   * problem
   */
  Planner(std::shared_ptr<ProblemDescription> problem);

  virtual void solve_(const std::vector<float> &start,
                      const std::vector<float> &end,
                      const Parameters &parameters,
                      PlannerSolution &recipient) = 0;

private:
  std::size_t
      state_space_size; // deduced from sampling a state with the samples

  std::mutex compute_solution_mtx;
};

} // namespace mt_rrt
