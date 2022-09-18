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

class MultiAgentPlanner : public MultiThreadedPlanner,
                          public SynchronizationAware {
public:
  enum class StarExpansionStrategyApproach { ExploitAllThreads, MonoThread };

  using MultiThreadedPlanner::MultiThreadedPlanner;

  void setStarApproach(StarExpansionStrategyApproach approach) {
    star_approach = approach;
  }
  StarExpansionStrategyApproach getStarApproach() const {
    return star_approach;
  }

protected:
  void solve_(const State &start, const State &end,
              const Parameters &parameters, PlannerSolution &recipient) final;

private:
  StarExpansionStrategyApproach star_approach =
      StarExpansionStrategyApproach::ExploitAllThreads;
};
} // namespace mt_rrt
