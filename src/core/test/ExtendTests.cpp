#include <gtest/gtest.h>

#include <MT-RRT/TreeBase.h>

#include "ExtendTestBase.h"

namespace mt_rrt::testing {
using TheExtender =
    ExtenderSingle<trivial_problem::Description<ExpansionStrategy::Single>,
                   TreeBase>;

TEST(ExtendSingleTest, nodes_deterministically_steered_only_once) {
  ExtendTestBase<TheExtender, ExpansionStrategy::Single> base;
  auto &[problem, start, end] = base.init(trivial_problem::Kind::Empty);
  problem.second.determinism.set(1.f);

  base.extender.emplace(problem, end.asView(), start.asView());

  extend_many(*base.extender, base.makeSearchPredicate());

  ASSERT_EQ(base.extender->getSolutions().size(), 1);

  bool solutions_ok =
      check_solutions(problem.first.connector->getChecker(),
                      base.extender->materializeAllSolutions(), start, end);
  ASSERT_TRUE(solutions_ok);
}

struct ExtendSingleTestFixture
    : ::testing::TestWithParam<trivial_problem::Kind> {};

TEST_P(ExtendSingleTestFixture, search) {
  ExtendTestBase<TheExtender, ExpansionStrategy::Single> base;
  auto &[problem, start, end] = base.init(GetParam());
  base.extender.emplace(problem, end.asView(), start.asView());

  extend_many(*base.extender, base.makeSearchPredicate());

  if (GetParam() == trivial_problem::Kind::NoSolution) {
    ASSERT_TRUE(base.extender->getSolutions().empty());
  } else {
    bool solutions_ok =
        check_solutions(problem.first.connector->getChecker(),
                        base.extender->materializeAllSolutions(), start, end);
    ASSERT_TRUE(solutions_ok);
  }
}

INSTANTIATE_TEST_CASE_P(ExtendSingleTest, ExtendSingleTestFixture,
                        ::testing::Values(trivial_problem::Kind::Empty,
                                          trivial_problem::Kind::NoSolution,
                                          trivial_problem::Kind::SmallObstacle,
                                          trivial_problem::Kind::Cluttered));
} // namespace mt_rrt::testing
