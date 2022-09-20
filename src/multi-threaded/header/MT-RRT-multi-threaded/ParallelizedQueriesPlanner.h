/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-multi-threaded/MultiThreadedPlanner.h>

namespace mt_rrt {
/**
 * @brief Approach described at Section "Parallelization of the query
 * activities" of the documentation
 */
class ParallelizedQueriesPlanner : public MultiThreadedPlanner {
public:
  using MultiThreadedPlanner::MultiThreadedPlanner;

protected:
  void solve_(const State &start, const State &end,
              const Parameters &parameters, PlannerSolution &recipient) final;
};
} // namespace mt_rrt
