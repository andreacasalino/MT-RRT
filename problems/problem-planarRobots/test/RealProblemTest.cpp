#include <gtest/gtest.h>

#include <LogResult.h>
#include <MT-RRT/StandardPlanner.h>
#include <PlanarRobotsProblem.h>
#include <PlanarRobotsProblemConversions.h>

using namespace mt_rrt;
using namespace mt_rrt::geom;
using namespace mt_rrt::robots;

using RealProblemTestFixture = ::testing::TestWithParam<ExpansionStrategy>;

TEST_P(RealProblemTestFixture, search_3_dof_problem) {
  std::shared_ptr<ProblemDescription> description;
  {
    Scene scene;

    std::vector<Joint> joints;
    joints.emplace_back().ray.set(0.1f);
    joints.emplace_back().ray.set(0.075f);
    joints.emplace_back().ray.set(0.05f);
    scene.robots.emplace_back(joints);

    auto &ob = scene.obstacles.emplace_back();
    ob.ray.set(0.25f);
    ob.center = geom::Point{2.f, 0};

    description = PosesConnector::make(2, scene);
  }
  StandardPlanner planner{std::move(*description)};

  std::vector<float> start{geom::PI * 0.25f, geom::PI * 0.25f,
                           -geom::PI * 0.25f};
  std::vector<float> end{-geom::PI * 0.25f, -geom::PI * 0.25f,
                         geom::PI * 0.25f};

  Parameters params;
  params.steer_trials.set(5);
  params.iterations.set(2000);
  std::string title;
  switch (GetParam()) {
  case ExpansionStrategy::Single:
    title = "single";
    break;
  case ExpansionStrategy::Bidir:
    title = "bidir";
    break;
  case ExpansionStrategy::Star:
    title = "star";
    break;
  }
  params.expansion_strategy = GetParam();

  auto solution = planner.solve(start, end, params);

  EXPECT_FALSE(solution.solution.empty());

  LogResult result;
  to_json(result,
          static_cast<const PosesConnector &>(*planner.problem().connector));
  result.addPlannerSolution(solution);
  result.addToScene("start") = start;
  result.addToScene("end") = end;
  Logger::get().add("three-dof", title, result.get());
}

INSTANTIATE_TEST_CASE_P(RealProblemTests, RealProblemTestFixture,
                        ::testing::Values(ExpansionStrategy::Single,
                                          ExpansionStrategy::Bidir,
                                          ExpansionStrategy::Star));
