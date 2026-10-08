#include <gtest/gtest.h>

#include <MT-RRT/TreeBase.h>

#include "ExtendTests.h"

namespace mt_rrt::testing {
using TheExtender = ExtenderBidirectional<
    trivial_problem::Description<ExpansionStrategy::Bidir>, TreeBase>;

using ExpansionStrategyBidirTest =
    ExtendTest<TheExtender, ExpansionStrategy::Bidir>;

TEST_F(ExpansionStrategyBidirTest, nodes_deterministically_steered_only_once) {
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

struct BidirStrategyFixture
    : ExtendTestBase<TheExtender, ExpansionStrategy::Bidir>,
      ::testing::TestWithParam<trivial_problem::Kind> {};

TEST_P(BidirStrategyFixture, search) {
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

INSTANTIATE_TEST_CASE_P(BidirStrategySearchTest, BidirStrategyFixture,
                        ::testing::Values(trivial_problem::Kind::Empty,
                                          trivial_problem::Kind::NoSolution,
                                          trivial_problem::Kind::SmallObstacle,
                                          trivial_problem::Kind::Cluttered));
} // namespace mt_rrt::testing
