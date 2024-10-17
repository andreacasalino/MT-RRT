#pragma once

#include <gtest/gtest.h>

#include <MT-RRT/extender/Extender.h>
#include <MiscConversions.h>
#include <TrivialProblemConversions.h>
#include <TestScenarios.h>
#include <LogResult.h>

namespace mt_rrt {
template<typename ExtenderT>
void log_test_case(const std::string &tag, const std::string &title,
                   ExtenderT &subject) {
  LogResult res;
  to_json(res, static_cast<const trivial::TrivialProblemConnector &>(
                   *subject.problem().connector));
  for (const auto &solution : subject.solutions) {
    res.addSolution(solution.materialize(), solution.cost());
  }
  std::vector<PlannerSolution::TreeSerialized> trees_serialized;
  subject.serializeTrees(trees_serialized);
  for (const auto &tree : trees_serialized) {
    res.addTree(tree);
  }
  Logger::get().add(tag, title, res.get());
}

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
      return ExtenderSingle<TreeHandler>(make_tree<TreeHandler>(
                                start.asView(), problem.point_problem,
                                problem.suggested_parameters),
                            end.asVec());
    } else {
      return ExtenderBidirectional<TreeHandler>(make_tree<TreeHandler>(
                                       start.asView(), problem.point_problem,
                                       problem.suggested_parameters),
                                   make_tree<TreeHandler>(
                                       end.asView(), problem.point_problem,
                                       problem.suggested_parameters));
    }
  }

  auto makeExtenderPtr() const {
    if constexpr (strategy == ExpansionStrategy::Single ||
                  strategy == ExpansionStrategy::Star) {
      return ExtenderSingle<TreeHandler>(
          make_tree<TreeHandler>(start.asView(), problem.point_problem,
                                        problem.suggested_parameters),
          end.asVec());
    } else {
      return ExtenderBidirectional<TreeHandler>(
          make_tree<TreeHandler>(start.asView(), problem.point_problem,
                                        problem.suggested_parameters),
          make_tree<TreeHandler>(end.asView(), problem.point_problem,
                                        problem.suggested_parameters));
    }
  }

  template<typename ExtenderT>
  void checkSolutions(const ExtenderT &extender) const {
    const auto &solutions = extender.solutions;
    ASSERT_FALSE(solutions.empty());
    EXPECT_TRUE(
        check_solutions(static_cast<const trivial::TrivialProblemConnector &>(
                            *problem.point_problem->connector),
                        solutions, start, end));
  }
};

std::string make_log_tag(trivial::Kind kind);
} // namespace mt_rrt
