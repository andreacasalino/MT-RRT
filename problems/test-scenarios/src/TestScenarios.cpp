#include <MT-RRT/Error.h>

#include <TestScenarios.h>

#include <algorithm>

namespace mt_rrt::trivial_problem {
bool is_a_collision_present(const trivial_problem::Checker &scenario,
                            const Solution &sequence) {
  auto it_prev = sequence.iter();
  auto it = sequence.iter();
  it.next();
  while (true) {
    auto previous = it_prev.next();
    auto current = it.next();
    if (!current) {
      break;
    }

    if (!scenario.check_from_to(*previous, *current)) {
      return true;
    }
  }
  return false;
}

namespace {
bool is_same(std::span<const float> a, std::span<const float> b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (int k = 0; k < a.size(); ++k) {
    if (a[k] != b[k]) {
      return false;
    }
  }
  return true;
}
} // namespace

bool check_solutions(const trivial_problem::Checker &scenario,
                     const std::vector<Solution> &solutions,
                     const geom::Point &start, const geom::Point &end) {
  for (const auto &sol : solutions) {
    std::size_t sol_size = sol.len();
    if ((sol_size < 2) || !is_same(sol.at(0), start.data()) ||
        !is_same(sol.at(sol_size - 1), end.data()) ||
        is_a_collision_present(scenario, sol)) {
      return false;
    }
  }
  return true;
}

bool check_loopy_connections(const Nodes &tree) {
  return std::any_of(tree.getNodes().begin(), tree.getNodes().end(),
                     [](const Node &n) {
                       try {
                         // if this does not throw means that
                         // the connections are ok
                         n.cost2Root();
                       } catch (const Error &) {
                         return true;
                       }
                       return false;
                     });
}

namespace {
geom::PointAllocated all_equals(float value) {
  return geom::PointAllocated{value, value};
}

std::tuple<geom::Boxes, SteerIterations, ProblemParameters,
           geom::PointAllocated, geom::PointAllocated>
make_empty_scenario() {
  ProblemParameters pars{Iterations{1500}, Determinism{0.15f}, false};
  return std::make_tuple(geom::Boxes{}, SteerIterations{3}, pars,
                         all_equals(-1.f), all_equals(1.f));
}

std::tuple<geom::Boxes, SteerIterations, ProblemParameters,
           geom::PointAllocated, geom::PointAllocated>
make_no_solution_scenario() {
  geom::PointAllocated obstacle_min_corner{-1.f / 3.f, -1.5f};
  geom::PointAllocated obstacle_max_corner{1.f / 3.f, 1.5f};

  ProblemParameters pars{Iterations{1500}, Determinism{0.15f}, false};
  geom::Boxes boxes{geom::Box{obstacle_min_corner, obstacle_max_corner}};

  return std::make_tuple(std::move(boxes), SteerIterations{3}, pars,
                         all_equals(-1.f), all_equals(1.f));
}

std::tuple<geom::Boxes, SteerIterations, ProblemParameters,
           geom::PointAllocated, geom::PointAllocated>
make_small_obstacle_scenario() {
  ProblemParameters pars{Iterations{1500}, Determinism{0.15f}, false};
  geom::Boxes boxes{geom::Box{all_equals(-0.8f), all_equals(0.8f)}};

  return std::make_tuple(std::move(boxes), SteerIterations{3}, pars,
                         all_equals(-1.f), all_equals(1.f));
}

std::tuple<geom::Boxes, SteerIterations, ProblemParameters,
           geom::PointAllocated, geom::PointAllocated>
make_cluttered_scenario() {
  ProblemParameters pars{Iterations{2000}, Determinism{0.15f}, false};
  geom::Boxes boxes;
  boxes.emplace_back(geom::Box{geom::PointAllocated{-1.f, -0.5f},
                               geom::PointAllocated{-0.5f, 0.5f}});
  boxes.emplace_back(geom::Box{geom::PointAllocated{0, -1.f},
                               geom::PointAllocated{1.f, -0.5f}});
  boxes.emplace_back(geom::Box{geom::PointAllocated{0, 0},
                               geom::PointAllocated{1.f / 3.f, 1.f}});
  boxes.emplace_back(geom::Box{geom::PointAllocated{1.f / 3.f, 2.f / 3.f},
                               geom::PointAllocated{2.f / 3.f, 1.f}});
  boxes.emplace_back(geom::Box{geom::PointAllocated{2.f / 3.f, 0},
                               geom::PointAllocated{1.f, 1.f / 3.f}});

  return std::make_tuple(std::move(boxes), SteerIterations{3}, pars,
                         all_equals(-1.f), all_equals(1.f));
}
} // namespace

std::tuple<geom::Boxes, SteerIterations, ProblemParameters,
           geom::PointAllocated, geom::PointAllocated>
make_scenario_data(Kind kind) {
  switch (kind) {
  case Kind::Empty:
    return make_empty_scenario();
  case Kind::NoSolution:
    return make_no_solution_scenario();
  case Kind::SmallObstacle:
    return make_small_obstacle_scenario();
  default: // case Kind::Cluttered:
    return make_cluttered_scenario();
  }
}
} // namespace mt_rrt::trivial_problem
