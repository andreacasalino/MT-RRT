////////////////////////////////////////////////////////////////
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //
// !!             have a look to ../ReadMe.md              !! //
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //
////////////////////////////////////////////////////////////////

// the class problem is defined here
#include <TrivialProblem2D.h>

#include <MT-RRT-core/StandardPlanner.h>

// just a bunch of utilities for generating log files
#include <Logger.h>

#include <iostream>

int main() {
  // build the connector, i.e. the object in charge of handling the rrt
  // extensions, containing the knowledge of the specific problem to solve
  //
  // in this class of examples, the TrivialProblem will be used, with
  // a single obstacle in the middle of the workspace
  auto static_data_problem =
      mt_rrt::samples::make_trivial_problem_2D_description(
          {mt_rrt::samples::Box2D{{-0.5f, -0.5f}, {0.5f, 0.5f}}});

  // define the parameters needed by any solver, that will be used later
  //
  // you can change the parameters as you prefer
  auto parameters = mt_rrt::Parameters{
      mt_rrt::ExpansionStrategy::Star, mt_rrt::SteerIterations{10},
      mt_rrt::Iterations{1000}, mt_rrt::Determinism{0.15f}, true};

  // build a planner that can be used to solve one or multiple problems
  //
  // you can use in the same way any other planner defined inside
  // src/multi-threaded/header/MT-RRT-multi-threaded
  mt_rrt::StandardPlanner planner(mt_rrt::ProblemDescription{
      std::move(static_data_problem->sampler),
      std::move(static_data_problem->connector), true,
      mt_rrt::Positive<float>{static_data_problem->gamma.get()}});

  /////////////////////////////////////////////////////////////////////////
  // find solution to the first problem using the generated planner
  /////////////////////////////////////////////////////////////////////////
  mt_rrt::State start({-1.f, -1.f});
  mt_rrt::State end({1.f, 1.f});
  std::cout << "Trying to connect start point: {" << start[0] << " , "
            << start[1] << "} : with end point: "
            << "{" << end[0] << " , " << end[1] << "}" << std::endl;
  auto solution_01 = planner.solve(start, end, parameters);
  std::cout << "A solution was ";
  if (solution_01.solution)
    std::cout << "found";
  else
    std::cout << "not found";
  std::cout << std::endl;
  std::cout << "To see the result ";
  // log results
  mt_rrt::utils::log_scenario(
      planner.problem(), solution_01,
      mt_rrt::samples::TrivialProblemConnectorLogger::LOGGER,
      mt_rrt::utils::DEFAULT_SOLUTION_LOGGER, "trivial-problem-scenario-01",
      mt_rrt::utils::make_python_show_sources(TRIVIAL_PROBLEM_PYTHON_SCRIPT));
  std::cout << std::endl << std::endl;

  /////////////////////////////////////////////////////////////////////////
  // find solution to the second problem using the generated planner
  /////////////////////////////////////////////////////////////////////////
  start = mt_rrt::State{-1.f, 1.f};
  end = mt_rrt::State{1.f, -1.f};
  std::cout << "Trying to connect start point: {" << start[0] << " , "
            << start[1] << "} : with end point: "
            << "{" << end[0] << " , " << end[1] << "}" << std::endl;
  auto solution_02 = planner.solve(start, end, parameters);
  std::cout << "A solution was ";
  if (solution_02.solution)
    std::cout << "found";
  else
    std::cout << "not found";
  std::cout << std::endl;
  std::cout << "To see the result ";
  // log results
  mt_rrt::utils::log_scenario(
      planner.problem(), solution_02,
      mt_rrt::samples::TrivialProblemConnectorLogger::LOGGER,
      mt_rrt::utils::DEFAULT_SOLUTION_LOGGER, "trivial-problem-scenario-01",
      mt_rrt::utils::make_python_show_sources(TRIVIAL_PROBLEM_PYTHON_SCRIPT));

  return EXIT_SUCCESS;
}

mt_rrt::samples::Boxes2D make_scenario_obstacles() {
  mt_rrt::samples::Rotator rotator{mt_rrt::samples::to_rad(-90.f)};

  std::vector<mt_rrt::samples::Boxes2D> islands_rotated;
  {
    auto &islands = islands_rotated.emplace_back();
    auto &vertical_rectangle =
        islands.emplace_back(mt_rrt::samples::Box2D{{80.f, 5.f}, {90.f, 80.f}});

    auto &horizontal_rectangle =
        islands.emplace_back(mt_rrt::samples::Box2D{vertical_rectangle});
    mt_rrt::samples::Rotator{mt_rrt::samples::to_rad(-90.f), {80.f, 80.f}}
        .rotate(horizontal_rectangle);
    mt_rrt::samples::traslate(horizontal_rectangle, {0, 10.f});

    islands.emplace_back(mt_rrt::samples::Box2D{{80.f, 80.f}, {90.f, 90.f}});

    islands.emplace_back(mt_rrt::samples::Box2D{{-35.f, 45.f}, {35.f, 55.f}});
  }
  for (std::size_t i = 1; i < 4; ++i) {
    islands_rotated.push_back(islands_rotated.back());
    rotator.rotate(islands_rotated.back());
  }

  mt_rrt::samples::Boxes2D result;
  for (const auto &island : islands_rotated) {
    result.insert(result.end(), island.begin(), island.end());
  }

  result.emplace_back(mt_rrt::samples::Box2D{{-30.f, -30.f}, {30.f, 30.f}});
  result.emplace_back(mt_rrt::samples::Box2D{{85.f, -110.f}, {110.f, -85.f}});
  result.emplace_back(mt_rrt::samples::Box2D{{-110.f, 85.f}, {-85.f, 110.f}});

  for (auto &box : result) {
    box.min_corner[0] /= 100.f;
    box.min_corner[1] /= 100.f;
    box.max_corner[0] /= 100.f;
    box.max_corner[1] /= 100.f;
  }
  return result;
}
