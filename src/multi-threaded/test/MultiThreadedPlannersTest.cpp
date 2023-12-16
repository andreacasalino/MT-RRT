#include <gtest/gtest.h>

#include <LogResult.h>
#include <TestScenarios.h>
#include <TrivialProblemConversions.h>

#include <MT-RRT/EmbarassinglyParallel.h>
#include <MT-RRT/LinkedTreesPlanner.h>
#include <MT-RRT/MultiAgentPlanner.h>
#include <MT-RRT/ParallelizedQueriesPlanner.h>
#include <MT-RRT/SharedTreePlanner.h>

#include <functional>

using namespace mt_rrt;

namespace {
struct OptimalSolutions {
  static OptimalSolutions &get() {
    static OptimalSolutions res = OptimalSolutions{};
    return res;
  }

  using OptimalSolution = std::vector<std::vector<float>>;

  bool check_optimality(
      mt_rrt::trivial::Kind kind,
      const std::vector<std::vector<float>> &found_solution) const {
    const auto &solutions = optimal_solutions.at(kind);
    return std::any_of(
        solutions.begin(), solutions.end(),
        [&found_solution = found_solution](const auto &candidate) {
          return mt_rrt::geom::curve_similarity(found_solution, candidate) <=
                 0.2f;
        });
  }

private:
  OptimalSolutions() {
    {
      auto &solutions = optimal_solutions[mt_rrt::trivial::Kind::SmallObstacle];
      solutions.emplace_back(
          OptimalSolution{{-1.f, -1.f}, {-0.8f, 0.8f}, {1.f, 1.f}});
      solutions.emplace_back(
          OptimalSolution{{-1.f, -1.f}, {0.8f, -0.8f}, {1.f, 1.f}});
    }
    {
      auto &solutions = optimal_solutions[mt_rrt::trivial::Kind::Cluttered];
      solutions.emplace_back(
          OptimalSolution{{-1.f, -1.f}, {1.f / 3.f, 0}, {1.f, 1.f}});
    }
  }

  std::unordered_map<mt_rrt::trivial::Kind, std::vector<OptimalSolution>>
      optimal_solutions;
};

std::string to_string(mt_rrt::trivial::Kind kind) {
  std::string result;
  switch (kind) {
  case mt_rrt::trivial::Kind::SmallObstacle:
    result = "OneObstacle";
    break;
  case mt_rrt::trivial::Kind::Cluttered:
    result = "Cluttered";
    break;
  default:
    throw mt_rrt::Error{"Invalid kind"};
    break;
  }
  return result;
}

std::string to_string(mt_rrt::ExpansionStrategy strtgy) {
  std::string result;
  switch (strtgy) {
  case mt_rrt::ExpansionStrategy::Single:
    result = "Single";
    break;
  case mt_rrt::ExpansionStrategy::Bidir:
    result = "Bidir";
    break;
  case mt_rrt::ExpansionStrategy::Star:
    result = "Star";
    break;
  }
  return result;
}

template <typename PlannerT, ExpansionStrategy Strategy> struct Info {
  using ThePlanner = PlannerT;
  static constexpr ExpansionStrategy TheStrategy = Strategy;
};

template <typename PlannerT>
using SetUpPred = std::function<void(PlannerT &, mt_rrt::Parameters &)>;

template <typename PlannerT> struct PlannerTrait {};

template <> struct PlannerTrait<EmbarassinglyParallelPlanner> {
  static const inline std::string plannerName = "EmbarassinglyParallel";
  static const inline SetUpPred<EmbarassinglyParallelPlanner> setUp =
      SetUpPred<EmbarassinglyParallelPlanner>{};
};

template <> struct PlannerTrait<ParallelizedQueriesPlanner> {
  static const inline std::string plannerName = "ParallelizedQueriesPlanner";
  static const inline SetUpPred<ParallelizedQueriesPlanner> setUp =
      SetUpPred<ParallelizedQueriesPlanner>{};
};

template <> struct PlannerTrait<SharedTreePlanner> {
  static const inline std::string plannerName = "SharedTreePlanner";
  static const inline SetUpPred<SharedTreePlanner> setUp =
      SetUpPred<SharedTreePlanner>{};
};

template <> struct PlannerTrait<LinkedTreesPlanner> {
  static const inline std::string plannerName = "LinkedTreesPlanner";
  static const inline SetUpPred<LinkedTreesPlanner> setUp =
      SetUpPred<LinkedTreesPlanner>{
          [](LinkedTreesPlanner &planner, Parameters &) {
            planner.synchronization().set(0.2f);
          }};
};

template <> struct PlannerTrait<MultiAgentPlanner> {
  static const inline std::string plannerName = "MultiAgentPlanner";
  static const inline SetUpPred<MultiAgentPlanner> setUp =
      SetUpPred<MultiAgentPlanner>{
          [](MultiAgentPlanner &planner, Parameters &) {
            planner.synchronization().set(0.2f);
          }};
};

template <typename InfoT>
class MultiThreadedPlannerTest : public ::testing::Test {
protected:
  const std::vector<float> START = std::vector<float>{-1.f, -1.f};
  const std::vector<float> END = std::vector<float>{1.f, 1.f};

  template <typename PlannerT, ExpansionStrategy TheStrategy>
  void solve(trivial::Kind kind) {
    auto problem = mt_rrt::trivial::make_scenario(kind, TheStrategy);
    PlannerT planner{problem.point_problem};
    planner.setThreads(2);
    if (PlannerTrait<PlannerT>::setUp) {
      PlannerTrait<PlannerT>::setUp(planner, problem.suggested_parameters);
    }
    auto solution =
        planner.solve(this->START, this->END, problem.suggested_parameters);

    const auto &connector =
        static_cast<const mt_rrt::trivial::TrivialProblemConnector &>(
            *planner.problem().connector);
    const auto &sequence = solution.solution;

    {
      // log results
      mt_rrt::LogResult res;
      to_json(res, connector);
      res.addSolution(sequence);
      for (const auto &tree : solution.trees) {
        res.addTree(*tree);
      }
      mt_rrt::Logger::get().add(PlannerTrait<PlannerT>::plannerName,
                                to_string(kind) + "_" + to_string(TheStrategy),
                                res.get());
    }

    ASSERT_TRUE(2 <= sequence.size());
    EXPECT_EQ(sequence.front(), this->START);
    EXPECT_EQ(sequence.back(), this->END);
    EXPECT_FALSE(is_a_collision_present(connector, sequence));
    if constexpr (TheStrategy == mt_rrt::ExpansionStrategy::Star) {
      EXPECT_TRUE(OptimalSolutions::get().check_optimality(kind, sequence));
    }
  }
};
} // namespace

using MultiThreadedPlannerTestTypes = testing::Types<

    Info<EmbarassinglyParallelPlanner, ExpansionStrategy::Single>,
    Info<EmbarassinglyParallelPlanner, ExpansionStrategy::Bidir>,
    Info<EmbarassinglyParallelPlanner, ExpansionStrategy::Star>,

    Info<ParallelizedQueriesPlanner, ExpansionStrategy::Single>,
    Info<ParallelizedQueriesPlanner, ExpansionStrategy::Bidir>,
    Info<ParallelizedQueriesPlanner, ExpansionStrategy::Star>,

    Info<SharedTreePlanner, ExpansionStrategy::Single>,
    Info<SharedTreePlanner, ExpansionStrategy::Bidir>,
    Info<SharedTreePlanner, ExpansionStrategy::Star>,

    Info<LinkedTreesPlanner, ExpansionStrategy::Single>,
    Info<LinkedTreesPlanner, ExpansionStrategy::Bidir>,
    Info<LinkedTreesPlanner, ExpansionStrategy::Star>,

    Info<MultiAgentPlanner, ExpansionStrategy::Single>,
    Info<MultiAgentPlanner, ExpansionStrategy::Star>

    >;
TYPED_TEST_CASE(MultiThreadedPlannerTest, MultiThreadedPlannerTestTypes);

TYPED_TEST(MultiThreadedPlannerTest, search_one_obstacle) {
  using Info = TypeParam;

  this->template solve<typename Info::ThePlanner, Info::TheStrategy>(
      trivial::Kind::SmallObstacle);
}

TYPED_TEST(MultiThreadedPlannerTest, search_cluttered) {
  using Info = TypeParam;

  this->template solve<typename Info::ThePlanner, Info::TheStrategy>(
      trivial::Kind::Cluttered);
}
