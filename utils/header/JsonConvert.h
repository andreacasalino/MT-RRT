/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include "../../src/core/src/Extender.h"
#include <MT-RRT-core/Planner.h>

#include <nlohmann/json.hpp>

namespace mt_rrt::utils {
void from_file(nlohmann::json &j, const std::string &fileName);

void to_json(nlohmann::json &j, const Tree &subject);

void to_json(nlohmann::json &j, const std::vector<Tree> &subject);

class ConnectorLogger {
public:
  ConnectorLogger() = default;
  virtual ~ConnectorLogger() = default;

  virtual void operator()(nlohmann::json &, const Connector &) const = 0;
};

template <typename ConnectorT>
class ConnectorLoggerTyped : public ConnectorLogger {
public:
  void operator()(nlohmann::json &j, const Connector &c) const final {
    const auto *as_ConnectorT = dynamic_cast<const ConnectorT *>(&c);
    if (nullptr == as_ConnectorT) {
      throw Error{"Invalid connector logger"};
    }
    log(j, *as_ConnectorT);
  }

protected:
  virtual void log(nlohmann::json &j, const ConnectorT &c) const = 0;
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
