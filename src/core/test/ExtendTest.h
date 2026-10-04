#pragma once

#include <gtest/gtest.h>

#include <MT-RRT/ExtenderBidir.h>
#include <MT-RRT/ExtenderSingle.h>
#include <MT-RRT/TreeBase.h>

#include <TestScenarios.h>

namespace mt_rrt {
template <ExpansionStrategy ExpansionStrategyT>
using TheExtenderSingle =
    mt_rrt::ExtenderSingle<trivial_problem::Description<ExpansionStrategyT>,
                           TreeBase>;

template <ExpansionStrategy ExpansionStrategyT>
using TheExtenderBidir =
    mt_rrt::ExtenderBidir<trivial_problem::Description<ExpansionStrategyT>,
                          TreeBase>;

template <typename E> void log_test_case(const E &subject);

template <ExpansionStrategy ExpansionStrategyT> class ExtendTest {
public:
  ExtendTest(trivial_problem::Kind kind)
      : problem{trivial::make_scenario<ExpansionStrategyT>(kind)} {}

  auto makeExtender() const {
    if constexpr (strategy == ExpansionStrategy::Single ||
                  strategy == ExpansionStrategy::Star) {
      return ExtenderSingle(std::make_unique<TreeHandlerBasic>(
                                start.asView(), problem.point_problem,
                                problem.suggested_parameters),
                            end.asVec());
    } else {
      return ExtenderBidirectional(std::make_unique<TreeHandlerBasic>(
                                       start.asView(), problem.point_problem,
                                       problem.suggested_parameters),
                                   std::make_unique<TreeHandlerBasic>(
                                       end.asView(), problem.point_problem,
                                       problem.suggested_parameters));
    }
  }

  void checkSolutions(const Extender &extender) const {
    const auto &solutions = extender.getSolutions();
    ASSERT_FALSE(solutions.empty());
    EXPECT_TRUE(
        check_solutions(static_cast<const trivial::TrivialProblemConnector &>(
                            *problem.point_problem->connector),
                        solutions, start, end));
  }

  ExtendProblem<ExpansionStrategyT> problem;
};
} // namespace mt_rrt
