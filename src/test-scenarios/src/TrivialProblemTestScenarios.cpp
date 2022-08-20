#include <MT-RRT-carpet/Error.h>

#include <Extender.h>
#include <TrivialProblemTestScenarios.h>

#include <algorithm>

namespace mt_rrt::utils {
TreeHandlerPtr make_tree_handler(const State &root_state,
                                 const ProblemDescriptionPtr &problem,
                                 const Parameters &parameters) {
  return std::make_unique<TreeHandler>(make_root(root_state), problem,
                                       parameters);
}

namespace {
float expected_cost(const std::vector<mt_rrt::State> &sequence) {
  float result = 0;
  for (std::size_t k = 1; k < sequence.size(); ++k) {
    result += euclidean_distance(sequence[k - 1], sequence[k]);
  }
  return result;
}
} // namespace

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

                        const auto expected = expected_cost(sequence);
                        if (fabs(expected - subject.first) > 0.01f) {
                          return true;
                        }

                        return is_a_collision_present(scenario, sequence);
                      });
}

namespace {
class Interpolator {
public:
  Interpolator(const std::vector<mt_rrt::State> &subject)
      : total_length(expected_cost(subject)) {
    for (std::size_t k = 1; k < subject.size(); ++k) {
      const auto &start = subject[k - 1];
      const auto &end = subject[k];
      segments.emplace_back(
          Segment{start, end, euclidean_distance(start, end)});
    }
  }

  State evaluate(const float s) const {
    float length = s * total_length;
    float cumulated_length = 0;
    for (const auto &segment : segments) {
      if (length <= (cumulated_length + segment.length)) {
        const float b_coeff = (length - cumulated_length) / segment.length;
        const float a_coeff = 1.f - b_coeff;
        State result;
        for (std::size_t k = 0; k < segment.end.size(); ++k) {
          result.push_back(a_coeff * segment.start[k] +
                           b_coeff * segment.end[k]);
        }
        return result;
      }
      cumulated_length += segment.length;
    }
    return segments.back().end;
  }

private:
  const float total_length;

  struct Segment {
    State start;
    State end;
    float length;
  };
  std::vector<Segment> segments;
};
} // namespace

float similarity(const std::vector<mt_rrt::State> &a,
                 const std::vector<mt_rrt::State> &b) {
  float result = 0;
  const float delta = 1.f / static_cast<float>(100);
  Interpolator interp_a(a);
  Interpolator interp_b(b);
  std::size_t counter = 0;
  for (float s = delta; s <= 1.f; s += delta, ++counter) {
    result += euclidean_distance(interp_a.evaluate(s), interp_b.evaluate(s));
  }
  return result / static_cast<float>(counter);
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
