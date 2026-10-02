/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Nodes.h>
#include <MT-RRT/Solution.h>
#include <MT-RRT/Types.h>

#include <chrono>
#include <optional>

namespace mt_rrt {
/**
 * @brief Groups all the information characterizing a found solution
 */
struct PlannerSolution {
  /**
   * @brief computation time spent for obtaining the solution, or trying to get
   * one.
   */
  std::chrono::nanoseconds time;

  /**
   * @brief iterations spent for obtaining the solution, or trying to get
   * one.
   */
  std::size_t iterations;

  /**
   * @brief The sequence of states forming the solution to the planning
   * problem. Is empty in case a solution was not found
   */
  std::optional<Solution> solution;

  struct ExtraInfo {
    std::vector<Solution> all_solutions;
    std::vector<Nodes> nodes;
  };
  std::optional<ExtraInfo> extra_info;
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
template <typename P>
concept Planner = requires(P obj, PlannerSolution &recipient,
                           std::span<const float> start,
                           std::span<const float> end) {
  {
    obj.solve(recipient, start, end)
    } -> std::same_as<
        std::pair<std::size_t /* iterations spent */, std::optional<Solution>>>;
};

template <Planner P>
void solve(P &planner, PlannerSolution &recipient, std::span<const float> start,
           std::span<const float> end) {
  recipient.extra_info.reset();

  std::chrono::steady_clock clck;
  auto tic = clck.now();
  auto &&[iterations, solution] = planner.solve(recipient, start, end);
  recipient.iterations = iterations;
  recipient.solution = std::move(solution);
  recipient.time =
      std::chrono::duration_cast<std::chrono::nanoseconds>(clck.now() - tic);
}
} // namespace mt_rrt
