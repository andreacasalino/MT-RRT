/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Planner.h>

namespace mt_rrt {
/**
 * @brief the classical mon-thread solver described in Section "Background on
 * RRT" of the documentation.
 */
class StandardPlanner : public Planner {
public:
  using Planner::Planner;

protected:
  void solve_(const std::vector<float> &start, const std::vector<float> &end,
              const Parameters &parameters, PlannerSolution &recipient) final;
};
} // namespace mt_rrt
