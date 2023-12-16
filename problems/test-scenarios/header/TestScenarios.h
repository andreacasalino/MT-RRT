#pragma once

#include <Geometry.h>
#include <MT-RRT/Extender.h>
#include <TrivialProblem.h>

namespace mt_rrt::trivial {
struct ExtendProblem {
  std::shared_ptr<ProblemDescription> point_problem;
  Parameters suggested_parameters;
  geom::Point start;
  geom::Point end;
};

enum class Kind { Empty, NoSolution, SmallObstacle, Cluttered };

ExtendProblem make_scenario(Kind kind, ExpansionStrategy strategy);

bool is_a_collision_present(const TrivialProblemConnector &scenario,
                            const std::vector<std::vector<float>> &sequence);

bool check_solutions(const TrivialProblemConnector &scenario,
                     const mt_rrt::Solutions &solutions,
                     const geom::Point &start, const geom::Point &end);

bool check_loopy_connections(const TreeHandler &tree);
} // namespace mt_rrt::trivial
