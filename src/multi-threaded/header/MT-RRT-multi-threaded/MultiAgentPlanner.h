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
 * @brief Approach described at Section "Multi agents approach" of
 * the documentation
 */
class MultiAgentPlanner : public MultiThreadedPlanner,
                          public SynchronizationAware {
public:
  /**
   * explanation:
   *  - ExploitAllThreads -> exploit all threads when gathering results from
   * slave. This affects significantly the performance only in case of Star
   * approach.
   *  - MonoThread -> results from slaves are gathered one at a time.
   */
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
