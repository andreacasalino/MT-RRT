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

class LinkedTreesPlanner : public MultiThreadedPlanner,
                           public SynchronizationAware {
public:
  LinkedTreesPlanner(ProblemDescription &&problem)
      : MultiThreadedPlanner(std::forward<ProblemDescription>(problem)){};

protected:
  void solve_(const State &start, const State &end,
              const Parameters &parameters, PlannerSolution &recipient) final;
};
} // namespace mt_rrt
