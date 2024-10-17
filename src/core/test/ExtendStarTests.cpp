#include "ExtendTest.h"

#include <map>

using namespace mt_rrt;
using namespace mt_rrt::trivial;

using StarStrategyFixture = ::testing::TestWithParam<Kind>;

TEST_P(StarStrategyFixture, search) {
  auto test = ExtendTest<ExpansionStrategy::Star>{GetParam()};
  auto extender = test.makeExtender();
  extender.search();

  test.checkSolutions(extender);

  {
    SCOPED_TRACE("check optimality");

    const auto best_solution = materialize_best(extender.solutions);
    using Sequence = std::vector<std::vector<float>>;
    std::vector<Sequence> optimal_paths;
    switch (GetParam()) {
    case Kind::Empty: {
      optimal_paths.emplace_back(
          Sequence{test.start.asVec(), test.end.asVec()});
    } break;
    case Kind::SmallObstacle: {
      optimal_paths.emplace_back(
          Sequence{test.start.asVec(), {-0.8f, 0.8f}, test.end.asVec()});
      optimal_paths.emplace_back(
          Sequence{test.start.asVec(), {0.8f, -0.8f}, test.end.asVec()});
    } break;
    case Kind::Cluttered: {
      optimal_paths.emplace_back(
          Sequence{test.start.asVec(), {1.f / 3.f, 0}, test.end.asVec()});
    } break;
    default:
      break;
    }
    bool is_optimal = std::any_of(
        optimal_paths.begin(), optimal_paths.end(),
        [&best_solution](const auto &candidate) {
          return geom::curve_similarity(best_solution, candidate) <= 0.2f;
        });
    EXPECT_TRUE(is_optimal);
  }

  mt_rrt::log_test_case("star", make_log_tag(GetParam()), extender);
}

INSTANTIATE_TEST_CASE_P(StarStrategySearchTest, StarStrategyFixture,
                        ::testing::Values(Kind::Empty, Kind::SmallObstacle,
                                          Kind::Cluttered));
