#include <gtest/gtest.h>

#include <MT-RRT/ExtenderSingle.h>
#include <MT-RRT/TreeBase.h>

#include <TestScenarios.h>

using namespace mt_rrt;
using namespace mt_rrt::trivial_problem;

// void log_test_case(const std::string &tag, const std::string &title,
//                    mt_rrt::Extender &subject) {
//   LogResult res;
//   to_json(res, static_cast<const trivial::TrivialProblemConnector &>(
//                    *subject.problem().connector));
//   for (const auto &solution : subject.getSolutions()) {
//     res.addSolution(*solution);
//   }
//   for (auto &&tree : subject.dumpTrees()) {
//     res.addTree(*tree);
//   }
//   Logger::get().add(tag, title, res.get());
// }

using TheDescription = trivial_problem::Description<ExpansionStrategy::Single>;

using TheExtender = ExtenderSingle<TheDescription, TreeBase>;

TEST(SingleStrategyTest, nodes_deterministically_steered_only_once) {
  auto &&[problem, start, end] =
      make_scenario<ExpansionStrategy::Single>(Kind::Empty);
  problem.second.determinism.set(1.f);

  TheExtender extender{problem, end.asView(), TreeBase{start.asView()}};

  extend_many(extender, std::make_shared<KeepSearchPredicate>());

  // extender.search();

  // const auto &solutions = extender.getSolutions();
  // ASSERT_EQ(solutions.size(), 1);
  // ASSERT_TRUE(check_solutions(static_cast<const TrivialProblemConnector &>(
  //                                 *problem.point_problem->connector),
  //                             solutions, start, end));

  // mt_rrt::log_test_case("single", "empty_only_deterministic", extender);
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
