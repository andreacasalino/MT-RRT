#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <TrivialProblemTestScenarios.h>
#ifdef TEST_LOGGING
#include "Log.h"
#endif

TEST_CASE("Single extender in an empty space",
          mt_rrt::merge(TEST_TAG, "[extend][single][empty]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;

  auto size = GENERATE(2, 5);

  auto scenario = make_empty_scenario(ExpansionStrategy::Single);
  const auto &start = scenario.start;
  const auto &end = scenario.end;

  SECTION("check that nodes are deterministically steered only once") {
    scenario.suggested_parameters.determinism.set(1.f);
    ExtenderSingle extender(make_tree_handler(start, scenario.point_problem,
                                              scenario.suggested_parameters),
                            end);
    extender.search();

#ifdef TEST_LOGGING
    log_test_case(extender, "single-empty_only_deterministic");
#endif

    const auto &solutions = extender.getSolutions();
    REQUIRE(solutions.size() == 1);
    REQUIRE(
        check_solutions(*dynamic_cast<const samples::TrivialProblemConnector *>(
                            scenario.point_problem->connector.get()),
                        solutions, start, end));
  }

  SECTION("check that at least a solution is found") {
    auto splitted_extend = GENERATE(true, false);

    std::unique_ptr<ExtenderSingle> extender;

    if (splitted_extend) {
      const std::size_t cycles = 10;
      scenario.suggested_parameters.iterations.set(
          scenario.suggested_parameters.iterations.get() / cycles);
      extender = std::make_unique<ExtenderSingle>(
          make_tree_handler(start, scenario.point_problem,
                            scenario.suggested_parameters),
          end);
      for (std::size_t k = 0; k < cycles; ++k) {
        extender->search();
      }
    } else {
      extender = std::make_unique<ExtenderSingle>(
          make_tree_handler(start, scenario.point_problem,
                            scenario.suggested_parameters),
          end);
      extender->search();
    }

#ifdef TEST_LOGGING
    log_test_case(*extender, "single-empty_many_solutions");
#endif

    const auto &solutions = extender->getSolutions();
    REQUIRE_FALSE(solutions.empty());
    REQUIRE(
        check_solutions(*dynamic_cast<const samples::TrivialProblemConnector *>(
                            scenario.point_problem->connector.get()),
                        solutions, start, end));
  }
}

TEST_CASE("Single extender with blocking obstacle",
          mt_rrt::merge(TEST_TAG, "[extend][single][blocking]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;

  auto size = GENERATE(2, 5);

  auto scenario = make_no_solution_scenario(size, ExpansionStrategy::Single);
  const auto &start = scenario.start;
  const auto &end = scenario.end;

  SECTION("check that at least a solution is found") {
    ExtenderSingle extender(make_tree_handler(start, scenario.point_problem,
                                              scenario.suggested_parameters),
                            end);
    extender.search();

#ifdef TEST_LOGGING
    log_test_case(extender, "single-no_solution");
#endif

    REQUIRE(extender.getSolutions().empty());
  }
}

TEST_CASE("Single extender with single obstacle",
          mt_rrt::merge(TEST_TAG, "[extend][single][obstacle]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;

  auto scenario = make_small_obstacle_scenario(ExpansionStrategy::Single);
  const auto &start = scenario.start;
  const auto &end = scenario.end;

  SECTION("check that at least a solution is found") {
    ExtenderSingle extender(make_tree_handler(start, scenario.point_problem,
                                              scenario.suggested_parameters),
                            end);
    extender.search();

#ifdef TEST_LOGGING
    log_test_case(extender, "single-one_obstacle");
#endif

    const auto &solutions = extender.getSolutions();
    REQUIRE_FALSE(solutions.empty());
    REQUIRE(
        check_solutions(*dynamic_cast<const samples::TrivialProblemConnector *>(
                            scenario.point_problem->connector.get()),
                        solutions, start, end));
  }
}

TEST_CASE("Single extender in cluttered scenario",
          mt_rrt::merge(TEST_TAG, "[extend][single][cluttered]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;

  auto scenario = make_cluttered_scenario(ExpansionStrategy::Single);
  const auto &start = scenario.start;
  const auto &end = scenario.end;

  SECTION("check that at least a solution is found") {
    ExtenderSingle extender(make_tree_handler(start, scenario.point_problem,
                                              scenario.suggested_parameters),
                            end);
    extender.search();

#ifdef TEST_LOGGING
    log_test_case(extender, "single-cluttered");
#endif

    const auto &solutions = extender.getSolutions();
    REQUIRE_FALSE(solutions.empty());
    REQUIRE(
        check_solutions(*dynamic_cast<const samples::TrivialProblemConnector *>(
                            scenario.point_problem->connector.get()),
                        solutions, start, end));
  }
}
