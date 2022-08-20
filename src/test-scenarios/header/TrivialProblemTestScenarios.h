#pragma once

#include "../../src/core/src/Extender.h"
// #include <MT-RRT-carpet/Strings.h>
#include <TrivialProblem.h>

namespace mt_rrt::utils {
TreeHandlerPtr make_tree_handler(const State &root_state,
                                 const ProblemDescriptionPtr &problem,
                                 const Parameters &parameters);

bool is_a_collision_present(const samples::TrivialProblemConnector &scenario,
                            const std::vector<State> &sequence);

bool check_solutions(const samples::TrivialProblemConnector &scenario,
                     const mt_rrt::Solutions &solutions, const State &start,
                     const State &end);

float similarity(const std::vector<mt_rrt::State> &a,
                 const std::vector<mt_rrt::State> &b);

bool check_loopy_connections(const Tree &tree);

struct ExtendProblem {
  std::shared_ptr<ProblemDescription> point_problem;
  Parameters suggested_parameters;
  State start;
  State end;
};

ExtendProblem make_empty_scenario(const ExpansionStrategy expansion_strategy);

ExtendProblem
make_no_solution_scenario(const std::size_t size,
                          const ExpansionStrategy expansion_strategy);

ExtendProblem
make_small_obstacle_scenario(const ExpansionStrategy expansion_strategy);

ExtendProblem
make_cluttered_scenario(const ExpansionStrategy expansion_strategy);
} // namespace mt_rrt::utils
