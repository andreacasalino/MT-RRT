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
 * @brief Approach described at Section "Parallel expansions on linked trees" of
 * the documentation
 */
class LinkedTreesPlanner : public MultiThreadedPlanner {
public:
  using MultiThreadedPlanner::MultiThreadedPlanner;

protected:
  void solve_(const std::vector<float> &start, const std::vector<float> &end,
              const Parameters &parameters, PlannerSolution &recipient) final;
};
} // namespace mt_rrt
