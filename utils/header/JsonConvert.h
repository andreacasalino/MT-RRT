/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include "../../src/core/src/Extender.h"
#include <MT-RRT-core/Planner.h>
#include <PointProblem.h>

#include <nlohmann/json.hpp>

namespace mt_rrt::utils {
void to_json(nlohmann::json &j, const Box &subject);

void to_json(nlohmann::json &j, const PointConnector &subject);

void to_json(nlohmann::json &j, const Tree &subject);

void to_json(nlohmann::json &j, const std::vector<Tree> &subject);

using ConnectorLogger =
    std::function<void(nlohmann::json &, const Connector &)>;

static const ConnectorLogger POINT_CONNECTOR_LOGGER =
    [](nlohmann::json &recipient, const Connector &point_problem) {
      const auto *as_point_connnector =
          dynamic_cast<const PointConnector *>(&point_problem);
      if (nullptr == as_point_connnector) {
        throw Error{"Not a point problem"};
      }
      recipient = *as_point_connnector;
    };

using SolutionLogger =
    std::function<void(nlohmann::json &, const std::vector<State> &)>;

static const SolutionLogger DEFAULT_SOLUTION_LOGGER =
    [](nlohmann::json &recipient, const std::vector<State> &sol) {
      recipient = sol;
    };

void to_json(nlohmann::json &j, const ProblemDescription &problem,
             const PlannerSolution &solution,
             const ConnectorLogger &connector_logger,
             const SolutionLogger &solution_logger = DEFAULT_SOLUTION_LOGGER);

void to_json(nlohmann::json &j, const Extender &subject,
             const ConnectorLogger &connector_logger,
             const SolutionLogger &solution_logger = DEFAULT_SOLUTION_LOGGER);
} // namespace mt_rrt::utils
