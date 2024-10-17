#include <MT-RRT/Error.h>

#include <TestScenarios.h>

#include <algorithm>

namespace mt_rrt::trivial {
bool is_a_collision_present(const TrivialProblemConnector &scenario,
                            const std::vector<std::vector<float>> &sequence) {
  for (std::size_t k = 1; k < sequence.size(); ++k) {
    geom::Point point_prev{View{sequence[k - 1]}};
    geom::Point point{View{sequence[k]}};
    for (const auto &box : scenario.getBoxes()) {
      if (box.collides(point_prev, point)) {
        return true;
      }
    }
  }
  return false;
}

bool check_loopy_connections(const TreeHandler &tree) {
  return std::find_if(tree.nodes.begin(), tree.nodes.end(), [](const Node *n) {
           try {
             n->cost2Root(); // if this does not throw means that
                             // the connections are ok
           } catch (const Error &) {
             return true;
           }
           return false;
         }) == tree.nodes.end();
}

namespace {
geom::Point all_equals(float value) { return geom::Point{value, value}; }

ExtendProblem make_empty_scenario(ExpansionStrategy expansion_strategy) {
  return ExtendProblem{TrivialProblemConnector::make(1, {}),
                       Parameters{expansion_strategy, SteerIterations{3},
                                  Iterations{1500}, Determinism{0.15f}, false, true},
                       all_equals(-1.f), all_equals(1.f)};
}

ExtendProblem make_no_solution_scenario(ExpansionStrategy expansion_strategy) {
  geom::Point obstacle_min_corner{-1.f / 3.f, -1.5f};
  geom::Point obstacle_max_corner{1.f / 3.f, 1.5f};
  return ExtendProblem{
      TrivialProblemConnector::make(
          1, geom::Boxes{geom::Box{obstacle_min_corner, obstacle_max_corner}}),
      Parameters{expansion_strategy, SteerIterations{3}, Iterations{1500},
                 Determinism{0.15f}, false, true},
      all_equals(-1.f), all_equals(1.f)};
}

ExtendProblem
make_small_obstacle_scenario(ExpansionStrategy expansion_strategy) {
  return ExtendProblem{
      TrivialProblemConnector::make(
          1, geom::Boxes{geom::Box{all_equals(-0.8f), all_equals(0.8f)}}),
      Parameters{expansion_strategy, SteerIterations{3}, Iterations{1500},
                 Determinism{0.15f}, false, true},
      all_equals(-1.f), all_equals(1.f)};
}

ExtendProblem make_cluttered_scenario(ExpansionStrategy expansion_strategy) {
  geom::Boxes obstacles;
  obstacles.emplace_back(geom::Box{{-1.f, -0.5f}, {-0.5f, 0.5f}});
  obstacles.emplace_back(geom::Box{{0, -1.f}, {1.f, -0.5f}});
  obstacles.emplace_back(geom::Box{{0, 0}, {1.f / 3.f, 1.f}});
  obstacles.emplace_back(geom::Box{{1.f / 3.f, 2.f / 3.f}, {2.f / 3.f, 1.f}});
  obstacles.emplace_back(geom::Box{{2.f / 3.f, 0}, {1.f, 1.f / 3.f}});

  return ExtendProblem{TrivialProblemConnector::make(1, std::move(obstacles)),
                       Parameters{expansion_strategy, SteerIterations{3},
                                  Iterations{2000}, Determinism{0.15f}, true, true},
                       all_equals(-1.f), all_equals(1.f)};
}
} // namespace

ExtendProblem make_scenario(Kind kind, ExpansionStrategy strategy) {
  ExtendProblem res;
  switch (kind) {
  case Kind::Empty:
    res = make_empty_scenario(strategy);
    break;
  case Kind::NoSolution:
    res = make_no_solution_scenario(strategy);
    break;
  case Kind::SmallObstacle:
    res = make_small_obstacle_scenario(strategy);
    break;
  case Kind::Cluttered:
    res = make_cluttered_scenario(strategy);
    break;
  }
  return res;
}
} // namespace mt_rrt::trivial
