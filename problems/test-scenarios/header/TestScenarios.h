#pragma once

#include <MT-RRT/Extender.h>

#include <Geometry.h>
#include <TrivialProblem.h>

namespace mt_rrt::trivial_problem {
enum class Kind { Empty, NoSolution, SmallObstacle, Cluttered };

template <ExpansionStrategy ExpansionStrategyT> struct ExtendProblem {
  Problem<Description<ExpansionStrategyT>> problem;
  geom::PointAllocated start;
  geom::PointAllocated end;
};

std::tuple<geom::Boxes, SteerIterations, ProblemParameters,
           geom::PointAllocated, geom::PointAllocated>
make_scenario_data(Kind kind);

template <ExpansionStrategy ExpansionStrategyT>
ExtendProblem<ExpansionStrategyT> make_scenario(Kind kind) {
  auto &&[boxes, steers, params, start, end] = make_scenario_data(kind);

  return {std::make_pair(
              Description<ExpansionStrategyT>{0, std::move(boxes), steers},
              std::move(params)),
          start, end};
}

bool is_a_collision_present(const trivial_problem::Checker &scenario,
                            const Solution &sequence);

bool check_solutions(const trivial_problem::Checker &scenario,
                     const std::vector<Solution> &solutions,
                     const geom::Point &start, const geom::Point &end);

bool check_loopy_connections(const Nodes &tree);
} // namespace mt_rrt::trivial_problem
