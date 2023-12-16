#include <gtest/gtest.h>

#include <LogResult.h>
#include <MT-RRT/StandardPlanner.h>
#include <NavigationProblem.h>
#include <NavigationProblemConversions.h>

using namespace mt_rrt;
using namespace mt_rrt::geom;
using namespace mt_rrt::navigation;

using RealProblemTestFixture = ::testing::TestWithParam<ExpansionStrategy>;

TEST_P(RealProblemTestFixture, search) {
  std::shared_ptr<ProblemDescription> description;
  {
    Scene scene{Cart{0.5f, 1.f, CartSteerLimits{0.5f, 1.f}}};

    auto *ob = &scene.obstacles.emplace_back();
    ob->ray.set(0.25f);
    ob->center = geom::Point{2.f, 0};

    ob = &scene.obstacles.emplace_back();
    ob->ray.set(0.25f);
    ob->center = geom::Point{2.f, 0.5f};

    ob = &scene.obstacles.emplace_back();
    ob->ray.set(0.25f);
    ob->center = geom::Point{2.f, 1.f};

    ob = &scene.obstacles.emplace_back();
    ob->ray.set(0.25f);
    ob->center = geom::Point{2.f, 1.5f};

    ob = &scene.obstacles.emplace_back();
    ob->ray.set(0.25f);
    ob->center = geom::Point{2.f, 4.f};

    description = CartPosesConnector::make(2, scene);
  }
  StandardPlanner planner{std::move(*description)};

  std::vector<float> start{-3.5f, 0, geom::to_rad(150)};
  std::vector<float> end{3.5f, 0, 0};

  Parameters params;
  params.steer_trials.set(5);
  params.iterations.set(1500);
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
  const auto &cartConnector =
      static_cast<const CartPosesConnector &>(*planner.problem().connector);
  to_json(result, cartConnector);
  solution.solution = interpolate(cartConnector, solution.solution);
  result.addPlannerSolution(solution);
  result.addToScene("start") = start;
  result.addToScene("end") = end;
  for (const auto &tree : solution.trees) {
    result.addTree(*tree);
  }
  Logger::get().add("cart", title, result.get());
}

INSTANTIATE_TEST_CASE_P(RealProblemTests, RealProblemTestFixture,
                        ::testing::Values(ExpansionStrategy::Single,
                                          ExpansionStrategy::Star));
