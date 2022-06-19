#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <Logger.h>
#include <PointProblemScenarios.h>

TEST_CASE("Star extender in an empty space",
          mt_rrt::merge(TEST_TAG, "[extend][star][empty]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;

  auto size = GENERATE(2, 4);

  auto scenario = make_empty_scenario(size, ExpansionStrategy::Star);
  const auto &start = scenario.start;
  const auto &end = scenario.end;

  SECTION("check rrt star optimality") {
    ExtenderSingle extender(make_tree_handler(start, scenario.point_problem,
                                              scenario.suggested_parameters),
                            end);
    extender.search();

#ifdef TEST_LOGGING
    log_scenario(extender, POINT_CONNECTOR_LOGGER, DEFAULT_SOLUTION_LOGGER,
                 "star-empty", POINT_PROBLEM_SHOW_PYTHON_SCRIPT);
#endif

    const auto &solutions = extender.getSolutions();
    REQUIRE_FALSE(solutions.empty());
    REQUIRE(check_solutions(*dynamic_cast<const PointConnector *>(
                                scenario.point_problem->connector.get()),
                            solutions, start, end));
    CHECK(check_loopy_connections(extender.dumpTrees().front()));
    CHECK(similarity(solutions.begin()->second->getSequence(), {start, end}) <=
          0.2f);
  }
}

TEST_CASE("Star extender with single obstacle",
          mt_rrt::merge(TEST_TAG, "[extend][star][obstacle]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;

  auto scenario = make_small_obstacle_scenario(ExpansionStrategy::Star);
  const auto &start = scenario.start;
  const auto &end = scenario.end;

  SECTION("check rrt star optimality") {
    ExtenderSingle extender(make_tree_handler(start, scenario.point_problem,
                                              scenario.suggested_parameters),
                            end);
    extender.search();

#ifdef TEST_LOGGING
    log_scenario(extender, POINT_CONNECTOR_LOGGER, DEFAULT_SOLUTION_LOGGER,
                 "star-one_obstacle", POINT_PROBLEM_SHOW_PYTHON_SCRIPT);
#endif

    const auto &solutions = extender.getSolutions();
    REQUIRE_FALSE(solutions.empty());
    REQUIRE(check_solutions(*dynamic_cast<const PointConnector *>(
                                scenario.point_problem->connector.get()),
                            solutions, start, end));
    CHECK(check_loopy_connections(extender.dumpTrees().front()));
    bool is_optimal = (similarity(solutions.begin()->second->getSequence(),
                                  {start, {-0.8f, 0.8f}, end}) <= 0.2f) ||
                      (similarity(solutions.begin()->second->getSequence(),
                                  {start, {0.8f, -0.8f}, end}) <= 0.2f);
    CHECK(is_optimal);
  }
}

TEST_CASE("Star extender in cluttered scenario",
          mt_rrt::merge(TEST_TAG, "[extend][star][cluttered]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;

  auto scenario = make_cluttered_scenario(ExpansionStrategy::Star);
  const auto &start = scenario.start;
  const auto &end = scenario.end;

  SECTION("check that at least a solution is found") {
    ExtenderSingle extender(make_tree_handler(start, scenario.point_problem,
                                              scenario.suggested_parameters),
                            end);
    extender.search();

#ifdef TEST_LOGGING
    log_scenario(extender, POINT_CONNECTOR_LOGGER, DEFAULT_SOLUTION_LOGGER,
                 "star-cluttered", POINT_PROBLEM_SHOW_PYTHON_SCRIPT);
#endif

    const auto &solutions = extender.getSolutions();
    REQUIRE_FALSE(solutions.empty());
    REQUIRE(check_solutions(*dynamic_cast<const PointConnector *>(
                                scenario.point_problem->connector.get()),
                            solutions, start, end));
    CHECK(check_loopy_connections(extender.dumpTrees().front()));
    CHECK(similarity(solutions.begin()->second->getSequence(),
                     {start, {1.f / 3.f, 0}, end}) <= 0.2f);
  }
}
