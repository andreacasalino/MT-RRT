/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/Planner.h>

namespace mt_rrt {
/**
 * @brief the classical mon-thread solver described in Section "Background on
 * RRT" of the documentation.
 */
class StandardPlanner : public Planner {
public:
  using Planner::Planner;

protected:
  void solve_(const State &start, const State &end,
              const Parameters &parameters, PlannerSolution &recipient) final;
};
} // namespace mt_rrt
