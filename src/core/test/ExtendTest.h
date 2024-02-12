#pragma once

#include <gtest/gtest.h>

#include <MT-RRT/ExtenderBidir.h>
#include <MT-RRT/ExtenderSingle.h>
#include <TestScenarios.h>

namespace mt_rrt {
void log_test_case(const std::string &tag, const std::string &title,
                   mt_rrt::Extender &subject);

template <ExpansionStrategy strategy> class ExtendTest {
public:
  ExtendTest(trivial::Kind kind)
      : problem{trivial::make_scenario(kind, strategy)}, start{problem.start},
        end{problem.end} {}

  trivial::ExtendProblem problem;
  const geom::Point &start;
  const geom::Point &end;

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

  auto makeExtenderPtr() const {
    if constexpr (strategy == ExpansionStrategy::Single ||
                  strategy == ExpansionStrategy::Star) {
      return std::make_unique<ExtenderSingle>(
          std::make_unique<TreeHandler>(start.asView(), problem.point_problem,
                                        problem.suggested_parameters),
          end.asVec());
    } else {
      return std::make_unique<ExtenderBidirectional>(
          std::make_unique<TreeHandler>(start.asView(), problem.point_problem,
                                        problem.suggested_parameters),
          std::make_unique<TreeHandler>(end.asView(), problem.point_problem,
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
};

std::string make_log_tag(trivial::Kind kind);
} // namespace mt_rrt
