#include <gtest/gtest.h>

#include <MT-RRT/TreeBase.h>

#include "ExtendTests.h"

namespace mt_rrt::testing {
using TheExtender =
    ExtenderSingle<trivial_problem::Description<ExpansionStrategy::Single>,
                   TreeBase>;

using ExpansionStrategySingleTest =
    ExtendTest<TheExtender, ExpansionStrategy::Single>;

TEST_F(ExpansionStrategySingleTest, nodes_deterministically_steered_only_once) {
  auto &[problem, start, end] = this->init(trivial_problem::Kind::Empty);
  problem.second.determinism.set(1.f);

  extender.emplace(problem, end.asView(), TreeBase{start.asView()});

  extend_many(*extender, makeSearchPredicate());

  ASSERT_EQ(extender->getSolutions().size(), 1);

  bool solutions_ok =
      check_solutions(problem.first.connector->getChecker(),
                      extender->materializeAllSolutions(), start, end);
  ASSERT_TRUE(solutions_ok);
}

struct SingleStrategyFixture
    : ExtendTestBase<TheExtender, ExpansionStrategy::Single>,
      ::testing::TestWithParam<trivial_problem::Kind> {};

TEST_P(SingleStrategyFixture, search) {
  auto &[problem, start, end] = this->init(trivial_problem::Kind::Empty);
  extender.emplace(problem, end.asView(), TreeBase{start.asView()});

  extend_many(*extender, makeSearchPredicate());

  if (GetParam() == trivial_problem::Kind::NoSolution) {
    ASSERT_TRUE(extender->getSolutions().empty());
  } else {
    bool solutions_ok =
        check_solutions(problem.first.connector->getChecker(),
                        extender->materializeAllSolutions(), start, end);
    ASSERT_TRUE(solutions_ok);
  }
}

INSTANTIATE_TEST_CASE_P(SingleStrategySearchTest, SingleStrategyFixture,
                        ::testing::Values(trivial_problem::Kind::Empty,
                                          trivial_problem::Kind::NoSolution,
                                          trivial_problem::Kind::SmallObstacle,
                                          trivial_problem::Kind::Cluttered));

/*
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
