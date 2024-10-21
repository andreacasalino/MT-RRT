#pragma once

#include <MT-RRT/ProblemDescription.h>
#include <MT-RRT/Solution.h>
#include <MT-RRT/TreeHandler.h>

#include <TrivialProblem.h>
#include <Geometry.h>

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

template<typename T>
bool check_solutions(const TrivialProblemConnector &scenario,
                     const mt_rrt::Solutions<T> &solutions,
                     const geom::Point &start, const geom::Point &end) {
  const auto start_vec = start.asVec();
  const auto end_vec = end.asVec();
  for (const auto &sol : solutions) {
    auto sequence = sol.materialize();
    if ((sequence.size() < 2) || (sequence.front() != start_vec) ||
        (sequence.back() != end_vec) ||
        is_a_collision_present(scenario, sequence)) {
      return false;
    }
  }
  return true;
}

bool check_loopy_connections(const TreeHandler &tree);
} // namespace mt_rrt::trivial
