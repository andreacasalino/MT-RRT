#include <MT-RRT-carpet/Error.h>

#include <Extender.h>
#include <Geometry.h>
#include <TrivialProblemTestScenarios.h>

#include <algorithm>

namespace mt_rrt::utils {
TreeHandlerPtr make_tree_handler(const State &root_state,
                                 const ProblemDescriptionPtr &problem,
                                 const Parameters &parameters) {
  return std::make_unique<TreeHandler>(make_root(root_state), problem,
                                       parameters);
}

bool is_a_collision_present(const samples::TrivialProblemConnector &scenario,
                            const std::vector<State> &sequence) {
  for (std::size_t k = 1; k < sequence.size(); ++k) {
    for (const auto &box : scenario.getBoxes()) {
      if (collides(sequence[k - 1], sequence[k], box)) {
        return true;
      }
    }
  }
  return false;
}

bool check_solutions(const samples::TrivialProblemConnector &scenario,
                     const mt_rrt::Solutions &solutions, const State &start,
                     const State &end) {
  return solutions.end() ==
         std::find_if(solutions.begin(), solutions.end(),
                      [&](const std::pair<float, SolutionPtr> &subject) {
                        auto sequence = subject.second->getSequence();

                        if ((sequence.size() < 2) ||
                            (sequence.front() != start) ||
                            (sequence.back() != end)) {
                          return true;
                        }

                        const auto expected = curve_length(sequence);
                        if (fabs(expected - subject.first) > 0.01f) {
                          return true;
                        }

                        return is_a_collision_present(scenario, sequence);
                      });
}

bool check_loopy_connections(const Tree &tree) {
  for (const auto &node : tree) {
    try {
      node->cost2Root(); // if this does not throw means that the connections
                         // are ok
    } catch (const Error &) {
      return false;
    }
  }
  return true;
}

namespace {
State all_equals(const std::size_t size, const float value) {
  State result;
  result.reserve(size);
  for (std::size_t k = 0; k < size; ++k) {
    result.push_back(value);
  }
  return result;
}
} // namespace

ExtendProblem make_empty_scenario(const ExpansionStrategy expansion_strategy) {
  return ExtendProblem{samples::make_trivial_problem_description(1, {}),
                       Parameters{expansion_strategy, SteerIterations{3},
                                  Iterations{1500}, Determinism{0.15f}, false},
                       all_equals(2, -1.f), all_equals(2, 1.f)};
}

ExtendProblem
make_no_solution_scenario(const std::size_t size,
                          const ExpansionStrategy expansion_strategy) {
  auto obstacle_min_corner = all_equals(size, -1.5f);
  obstacle_min_corner[0] = -1.f / 3.f;
  auto obstacle_max_corner = all_equals(size, 1.5f);
  obstacle_max_corner[0] = 1.f / 3.f;
  return ExtendProblem{samples::make_trivial_problem_description(
                           1, samples::Boxes{samples::Box{
                                  obstacle_min_corner, obstacle_max_corner}}),
                       Parameters{expansion_strategy, SteerIterations{3},
                                  Iterations{1500}, Determinism{0.15f}, false},
                       all_equals(size, -1.f), all_equals(size, 1.f)};
}

ExtendProblem
make_small_obstacle_scenario(const ExpansionStrategy expansion_strategy) {
  return ExtendProblem{samples::make_trivial_problem_description(
                           1, samples::Boxes{samples::Box{
                                  all_equals(2, -0.8f), all_equals(2, 0.8f)}}),
                       Parameters{expansion_strategy, SteerIterations{3},
                                  Iterations{1500}, Determinism{0.15f}, false},
                       all_equals(2, -1.f), all_equals(2, 1.f)};
}

ExtendProblem
make_cluttered_scenario(const ExpansionStrategy expansion_strategy) {
  samples::Boxes obstacles;
  obstacles.emplace_back(samples::Box{{-1.f, -0.5f}, {-0.5f, 0.5f}});
  obstacles.emplace_back(samples::Box{{0, -1.f}, {1.f, -0.5f}});
  obstacles.emplace_back(samples::Box{{0, 0}, {1.f / 3.f, 1.f}});
  obstacles.emplace_back(
      samples::Box{{1.f / 3.f, 2.f / 3.f}, {2.f / 3.f, 1.f}});
  obstacles.emplace_back(samples::Box{{2.f / 3.f, 0}, {1.f, 1.f / 3.f}});

  return ExtendProblem{
      samples::make_trivial_problem_description(1, std::move(obstacles)),
      Parameters{expansion_strategy, SteerIterations{3}, Iterations{2000},
                 Determinism{0.15f}, true},
      all_equals(2, -1.f), all_equals(2, 1.f)};
}
} // namespace mt_rrt::utils
