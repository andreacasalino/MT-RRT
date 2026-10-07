#include <gtest/gtest.h>

#include <MT-RRT/TreeBase.h>

#include "ExtendTests.h"

namespace mt_rrt::testing {
using TheExtendTest = ExtendTest<ExtenderSingle<
    trivial_problem::Description<ExpansionStrategy::Single>, TreeBase>>;

TEST_F(TheExtendTest, nodes_deterministically_steered_only_once) {
  auto [problem, start, end] =
      this->init<ExpansionStrategy::Single>(trivial_problem::Kind::Empty);
  problem.second.determinism.set(1.f);

  extender.emplace(problem, end.asView(), TreeBase{start.asView()});

  extend_many(*extender,
              std::make_shared<KeepSearchPredicate<ExpansionStrategy::Single>>(
                  problem.second));

  const auto &solutions = extender->getSolutions();
  ASSERT_EQ(solutions.size(), 1);
  bool solutions_ok =
      check_solutions(problem.first.connector->getChecker(),
                      extender->materializeAllSolutions(), start, end);
  ASSERT_TRUE(solutions_ok);
}

/*
using SingleStrategyFixture = ::testing::TestWithParam<Kind>;

TEST_P(SingleStrategyFixture, search) {
  auto test = ExtendTest<ExpansionStrategy::Single>{GetParam()};
  auto extender = test.makeExtender();
  extender.search();

  if (GetParam() == Kind::NoSolution) {
    ASSERT_TRUE(extender.getSolutions().empty());
  } else {
    test.checkSolutions(extender);
  }

  mt_rrt::log_test_case("single", make_log_tag(GetParam()), extender);
}

INSTANTIATE_TEST_CASE_P(SingleStrategySearchTest, SingleStrategyFixture,
                        ::testing::Values(Kind::Empty, Kind::NoSolution,
                                          Kind::SmallObstacle,
                                          Kind::Cluttered));

TEST_F(SingleStrategyTest, multiple_search_cycles) {
  const std::size_t cycles = 10;
  problem.suggested_parameters.iterations.set(
      problem.suggested_parameters.iterations.get() / cycles);
  auto extender = makeExtender();
  for (std::size_t k = 0; k < cycles; ++k)
    extender.search();

  checkSolutions(extender);

  mt_rrt::log_test_case("single", "multiple_cycles", extender);
}
*/
} // namespace mt_rrt::testing
