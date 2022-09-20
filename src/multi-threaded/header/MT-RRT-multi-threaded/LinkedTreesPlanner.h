/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-multi-threaded/MultiThreadedPlanner.h>
#include <MT-RRT-multi-threaded/Synchronization.h>

namespace mt_rrt {
/**
 * @brief Approach described at Section "Parallel expansions on linked trees" of
 * the documentation
 */
class LinkedTreesPlanner : public MultiThreadedPlanner,
                           public SynchronizationAware {
public:
  using MultiThreadedPlanner::MultiThreadedPlanner;

protected:
  void solve_(const State &start, const State &end,
              const Parameters &parameters, PlannerSolution &recipient) final;
};
} // namespace mt_rrt
