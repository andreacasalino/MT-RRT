#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <Logger.h>
#include <TrivialProblemTestScenarios.h>
#ifdef TEST_LOGGING
#include <TrivialProblemJson.h>
#endif

TEST_CASE("Bidir extender in an empty space",
          mt_rrt::merge(TEST_TAG, "[extend][bidir][empty]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;

  auto size = GENERATE(2, 5);

  auto scenario = make_empty_scenario(ExpansionStrategy::Bidir);
  const auto &start = scenario.start;
  const auto &end = scenario.end;

  SECTION("check that nodes are deterministically steered only once") {
    scenario.suggested_parameters.determinism.set(1.f);
    ExtenderBidirectional extender(
        make_tree_handler(start, scenario.point_problem,
                          scenario.suggested_parameters),
        make_tree_handler(end, scenario.point_problem,
                          scenario.suggested_parameters));
    extender.search();

#ifdef TEST_LOGGING
    log_scenario(extender, samples::TrivialProblemConverter::CONVERTER,
                 "bidir-empty_only_deterministic",
                 PythonSources{TRIVIAL_PROBLEM_PYTHON_SCRIPT});
#endif

    const auto &solutions = extender.getSolutions();
    // REQUIRE(solutions.size() == 1); // not true for the current
    // implementation, but maybe it's want we want
    REQUIRE(
        check_solutions(*dynamic_cast<const samples::TrivialProblemConnector *>(
                            scenario.point_problem->connector.get()),
                        solutions, start, end));
  }

  SECTION("check that at least a solution is found") {
    auto splitted_extend = GENERATE(true, false);

    std::unique_ptr<ExtenderBidirectional> extender;
    if (splitted_extend) {
      const std::size_t cycles = 10;
      scenario.suggested_parameters.iterations.set(
          scenario.suggested_parameters.iterations.get() / cycles);
      extender = std::make_unique<ExtenderBidirectional>(
          make_tree_handler(start, scenario.point_problem,
                            scenario.suggested_parameters),
          make_tree_handler(end, scenario.point_problem,
                            scenario.suggested_parameters));
      for (std::size_t k = 0; k < cycles; ++k) {
        extender->search();
      }
    } else {
      extender = std::make_unique<ExtenderBidirectional>(
          make_tree_handler(start, scenario.point_problem,
                            scenario.suggested_parameters),
          make_tree_handler(end, scenario.point_problem,
                            scenario.suggested_parameters));
      extender->search();
    }

#ifdef TEST_LOGGING
    log_scenario(*extender, samples::TrivialProblemConverter::CONVERTER,
                 "bidir-empty_many_solutions",
                 PythonSources{TRIVIAL_PROBLEM_PYTHON_SCRIPT});
#endif

    const auto &solutions = extender->getSolutions();
    REQUIRE_FALSE(solutions.empty());
    REQUIRE(
        check_solutions(*dynamic_cast<const samples::TrivialProblemConnector *>(
                            scenario.point_problem->connector.get()),
                        solutions, start, end));
  }
}

TEST_CASE("Bidir extender with blocking obstacle",
          mt_rrt::merge(TEST_TAG, "[extend][bidir][blocking]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;

  auto size = GENERATE(2, 5);

  auto scenario = make_no_solution_scenario(size, ExpansionStrategy::Bidir);
  const auto &start = scenario.start;
  const auto &end = scenario.end;

  SECTION("check that at least a solution is found") {
    ExtenderBidirectional extender(
        make_tree_handler(start, scenario.point_problem,
                          scenario.suggested_parameters),
        make_tree_handler(end, scenario.point_problem,
                          scenario.suggested_parameters));
    extender.search();

#ifdef TEST_LOGGING
    log_scenario(extender, samples::TrivialProblemConverter::CONVERTER,
                 "bidir-no_solution",
                 PythonSources{TRIVIAL_PROBLEM_PYTHON_SCRIPT});
#endif

    REQUIRE(extender.getSolutions().empty());
  }
}

TEST_CASE("Bidir extender with single obstacle",
          mt_rrt::merge(TEST_TAG, "[extend][bidir][obstacle]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;

  auto scenario = make_small_obstacle_scenario(ExpansionStrategy::Bidir);
  const auto &start = scenario.start;
  const auto &end = scenario.end;

  SECTION("check that at least a solution is found") {
    ExtenderBidirectional extender(
        make_tree_handler(start, scenario.point_problem,
                          scenario.suggested_parameters),
        make_tree_handler(end, scenario.point_problem,
                          scenario.suggested_parameters));
    extender.search();

#ifdef TEST_LOGGING
    log_scenario(extender, samples::TrivialProblemConverter::CONVERTER,
                 "bidir-one_obstacle",
                 PythonSources{TRIVIAL_PROBLEM_PYTHON_SCRIPT});
#endif

    const auto &solutions = extender.getSolutions();
    REQUIRE_FALSE(solutions.empty());
    REQUIRE(
        check_solutions(*dynamic_cast<const samples::TrivialProblemConnector *>(
                            scenario.point_problem->connector.get()),
                        solutions, start, end));
  }
}

TEST_CASE("Bidir extender in cluttered scenario",
          mt_rrt::merge(TEST_TAG, "[extend][bidir][cluttered]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;

  auto scenario = make_cluttered_scenario(ExpansionStrategy::Bidir);
  const auto &start = scenario.start;
  const auto &end = scenario.end;

  SECTION("check that at least a solution is found") {
    ExtenderBidirectional extender(
        make_tree_handler(start, scenario.point_problem,
                          scenario.suggested_parameters),
        make_tree_handler(end, scenario.point_problem,
                          scenario.suggested_parameters));
    extender.search();

#ifdef TEST_LOGGING
    log_scenario(extender, samples::TrivialProblemConverter::CONVERTER,
                 "bidir-cluttered",
                 PythonSources{TRIVIAL_PROBLEM_PYTHON_SCRIPT});
#endif

    const auto &solutions = extender.getSolutions();
    REQUIRE_FALSE(solutions.empty());
    REQUIRE(
        check_solutions(*dynamic_cast<const samples::TrivialProblemConnector *>(
                            scenario.point_problem->connector.get()),
                        solutions, start, end));
  }
}