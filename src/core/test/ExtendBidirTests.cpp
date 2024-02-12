#include "ExtendTest.h"

using namespace mt_rrt;
using namespace mt_rrt::trivial;

struct BidirStrategyTest : public ::testing::Test,
                           public ExtendTest<ExpansionStrategy::Bidir> {
  BidirStrategyTest() : ExtendTest<ExpansionStrategy::Bidir>{Kind::Empty} {}
};

TEST_F(BidirStrategyTest,
       DISABLED_nodes_deterministically_steered_only_once) { // not true in the
                                                             // current
                                                             // implementation
  problem.suggested_parameters.determinism.set(1.f);
  auto extender = makeExtender();
  extender.search();

  const auto &solutions = extender.getSolutions();
  ASSERT_EQ(solutions.size(), 1);
  ASSERT_TRUE(check_solutions(static_cast<const TrivialProblemConnector &>(
                                  *problem.point_problem->connector),
                              solutions, start, end));

  mt_rrt::log_test_case("bidir", "empty_only_deterministic", extender);
}

using BidirStrategyFixture = ::testing::TestWithParam<Kind>;

TEST_P(BidirStrategyFixture, search) {
  auto test = ExtendTest<ExpansionStrategy::Bidir>{GetParam()};
  auto extender = test.makeExtender();
  extender.search();

  if (GetParam() == Kind::NoSolution) {
    ASSERT_TRUE(extender.getSolutions().empty());
  } else {
    test.checkSolutions(extender);
  }

  mt_rrt::log_test_case("bidir", make_log_tag(GetParam()), extender);
}

INSTANTIATE_TEST_CASE_P(BidirStrategySearchTest, BidirStrategyFixture,
                        ::testing::Values(Kind::Empty, Kind::NoSolution,
                                          Kind::SmallObstacle,
                                          Kind::Cluttered));

TEST_F(BidirStrategyTest, multiple_search_cycles) {
  const std::size_t cycles = 10;
  problem.suggested_parameters.iterations.set(
      problem.suggested_parameters.iterations.get() / cycles);
  auto extender = makeExtender();
  for (std::size_t k = 0; k < cycles; ++k)
    extender.search();

  checkSolutions(extender);

  mt_rrt::log_test_case("bidir", "multiple_cycles", extender);
}
