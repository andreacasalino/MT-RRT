/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include "../../src/core/src/Extender.h"
#include <MT-RRT-core/Planner.h>

#include <JsonConvert.h>

#include <optional>
#include <string>
#include <unordered_map>

namespace mt_rrt::utils {
class Logger {
public:
  static void
  log(const std::string &tag, const nlohmann::json &content,
      const std::optional<std::string> &python_script = std::nullopt);

private:
  static std::unordered_map<std::string, std::size_t> counters;
};

void log_scenario(
    const ProblemDescription &problem, const PlannerSolution &solution,
    const ConnectorLogger &connector_logger,
    const SolutionLogger &solution_logger, const std::string &case_name,
    const std::optional<std::string> &python_visualization_script);

void log_scenario(
    const mt_rrt::Extender &subject, const ConnectorLogger &connector_logger,
    const SolutionLogger &solution_logger, const std::string &case_name,
    const std::optional<std::string> &python_visualization_script);
} // namespace mt_rrt::utils
