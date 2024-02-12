/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/MultiThreadedPlanner.h>

namespace mt_rrt {
/**
 * @brief Into N different threads, N different solution are found running the
 * classical version of the planner. The best solution among thosw found by all
 * threads is returned.
 */
class EmbarassinglyParallelPlanner : public MultiThreadedPlanner {
public:
  using MultiThreadedPlanner::MultiThreadedPlanner;

protected:
  void solve_(const std::vector<float> &start, const std::vector<float> &end,
              const Parameters &parameters, PlannerSolution &recipient) final;
};
} // namespace mt_rrt
