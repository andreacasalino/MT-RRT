#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <MT-RRT-multi-threaded/MultiThreadedPlanner.h>

#include <Geometry.h>
#include <Logger.h>
#include <TrivialProblemTestScenarios.h>
#ifdef TEST_LOGGING
#include <TrivialProblemJson.h>
#endif

#include <algorithm>

namespace {
enum class ScenarioKind { SingleObstacle, Cluttered };

static constexpr std::size_t THREADS_TO_USE = 2; // 4;

struct ScenarioHelper {
  ScenarioKind kind;
  mt_rrt::ExpansionStrategy strategy;

  mt_rrt::utils::ExtendProblem make_scenario() const {
    switch (kind) {
    case ScenarioKind::SingleObstacle:
      return mt_rrt::utils::make_small_obstacle_scenario(strategy);
    case ScenarioKind::Cluttered:
    default:
      return mt_rrt::utils::make_cluttered_scenario(strategy);
    }
  };

  std::vector<std::vector<mt_rrt::State>> possible_optimal_solutions() const {
    switch (kind) {
    case ScenarioKind::SingleObstacle:
      return {{{-1.f, -1.f}, {-0.8f, 0.8f}, {1.f, 1.f}},
              {{-1.f, -1.f}, {0.8f, -0.8f}, {1.f, 1.f}}};
    case ScenarioKind::Cluttered:
    default:
      return {{{-1.f, -1.f}, {1.f / 3.f, 0}, {1.f, 1.f}}};
    }
  }
};

bool check_optimality(
    const std::vector<mt_rrt::State> &found_solution,
    const std::vector<std::vector<mt_rrt::State>> &possible_optimal_solutions) {
  return std::any_of(
      possible_optimal_solutions.begin(), possible_optimal_solutions.end(),
      [&found_solution](const std::vector<mt_rrt::State> &candidate) {
        return mt_rrt::utils::curve_similarity(found_solution, candidate) <=
               0.2f;
      });
}

template <typename PlannerT, typename ExpandStrategies, typename PlannerTSetter>
void check_planner_with_setter(const std::string &log_tag,
                               ExpandStrategies strategy,
                               const PlannerTSetter &setter) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;

  auto scenario_kind =
      GENERATE(ScenarioKind::SingleObstacle, ScenarioKind::Cluttered);

  ScenarioHelper helper{scenario_kind, strategy};

  auto scenario = helper.make_scenario();
  const auto &start = scenario.start;
  const auto &end = scenario.end;
  PlannerT planner(
      ProblemDescription{std::move(scenario.point_problem->sampler),
                         std::move(scenario.point_problem->connector),
                         scenario.point_problem->simmetry,
                         Positive<float>{scenario.point_problem->gamma.get()}});
  planner.setThreads(Threads{THREADS_TO_USE});
  auto params = scenario.suggested_parameters;
  setter(planner, params);
  auto solution = planner.solve(start, end, params);

#ifdef TEST_LOGGING
  log_scenario(
      planner.problem(), solution, samples::TrivialProblemConverter::CONVERTER,
      log_tag,
      mt_rrt::utils::make_python_show_sources(TRIVIAL_PROBLEM_PYTHON_SCRIPT));
#endif

  REQUIRE(solution.solution);
  const auto &sequence = solution.solution.value();
  REQUIRE(2 <= sequence.size());
  CHECK(sequence.front() == start);
  CHECK(sequence.back() == end);
  CHECK_FALSE(is_a_collision_present(
      static_cast<const samples::TrivialProblemConnector &>(
          *planner.problem().connector),
      sequence));
  if (helper.strategy == ExpansionStrategy::Star) {
    // the solution found is close to one of the optimal?
    CHECK(check_optimality(sequence, helper.possible_optimal_solutions()));
  }

  // TODO check time is lower than serial version
}

template <typename PlannerT, typename ExpandStrategies>
void check_planner(const std::string &log_tag, ExpandStrategies strategy) {
  check_planner_with_setter<PlannerT, ExpandStrategies>(
      log_tag, std::forward<ExpandStrategies>(strategy),
      [](PlannerT &, mt_rrt::Parameters &) {});
}
} // namespace

#include <MT-RRT-multi-threaded/EmbarassinglyParallel.h>
TEST_CASE("Embarassingly parallel planner",
          mt_rrt::merge(TEST_TAG, "[solver][embarassingly-parallel]")) {
  check_planner<mt_rrt::EmbarassinglyParallelPlanner>(
      "embarassingly-parallel", GENERATE(mt_rrt::ExpansionStrategy::Single,
                                         mt_rrt::ExpansionStrategy::Bidir,
                                         mt_rrt::ExpansionStrategy::Star));
}

#include <MT-RRT-multi-threaded/ParallelizedQueriesPlanner.h>
TEST_CASE("Parallelized queries planner",
          mt_rrt::merge(TEST_TAG, "[solver][parallel-queries]")) {
  check_planner<mt_rrt::ParallelizedQueriesPlanner>(
      "parallel-queries", GENERATE(mt_rrt::ExpansionStrategy::Single,
                                   mt_rrt::ExpansionStrategy::Bidir,
                                   mt_rrt::ExpansionStrategy::Star));
}

/*
#include <MT-RRT-multi-threaded/SharedTreePlanner.h>
TEST_CASE("Shared tree planner", mt_rrt::merge(TEST_TAG, "[solver][shared-tree]")) {
  check_planner<mt_rrt::SharedTreePlanner>(
      "shared-tree", GENERATE(mt_rrt::ExpansionStrategy::Single,
                              mt_rrt::ExpansionStrategy::Bidir,
                              mt_rrt::ExpansionStrategy::Star));
}

#include <MT-RRT-multi-threaded/LinkedTreesPlanner.h>
TEST_CASE("Linked trees planner", mt_rrt::merge(TEST_TAG, "[solver][linked-trees]")) {
  check_planner<mt_rrt::LinkedTreesPlanner>(
      "linked-trees", GENERATE(mt_rrt::ExpansionStrategy::Single,
                               mt_rrt::ExpansionStrategy::Bidir,
                               mt_rrt::ExpansionStrategy::Star));
}
*/

#include <MT-RRT-multi-threaded/MultiAgentPlanner.h>
TEST_CASE("Multi agent planner", mt_rrt::merge(TEST_TAG, "[solver][multi-agent]")) {
  check_planner_with_setter<mt_rrt::MultiAgentPlanner>(
      "multi-agent",
      GENERATE(mt_rrt::ExpansionStrategy::Single,
               mt_rrt::ExpansionStrategy::Star),
      [](mt_rrt::MultiAgentPlanner &planner, mt_rrt::Parameters &params) {
        planner.synchronization().set(0.25f);
        params.iterations.set(params.iterations.get() * 4);
      });
}

TEST_CASE("Single threaded star approach multi agent planner",
          mt_rrt::merge(TEST_TAG, "[solver][multi-agent]")) {
  check_planner_with_setter<mt_rrt::MultiAgentPlanner>(
      "multi-agent", GENERATE(mt_rrt::ExpansionStrategy::Star),
      [](mt_rrt::MultiAgentPlanner &planner, mt_rrt::Parameters &params) {
        planner.synchronization().set(0.25f);
        params.iterations.set(params.iterations.get() * 4);
        planner.setStarApproach(mt_rrt::MultiAgentPlanner::
                                    StarExpansionStrategyApproach::MonoThread);
      });
}
