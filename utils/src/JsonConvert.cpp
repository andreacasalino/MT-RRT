/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <JsonConvert.h>

namespace mt_rrt::utils {
namespace {
void convert(nlohmann::json &j, const State &start, const State &end) {
  j["start"] = start;
  j["end"] = end;
}
} // namespace

void to_json(nlohmann::json &j, const Box &subject) {
  convert(j, subject.min_corner, subject.max_corner);
}

void to_json(nlohmann::json &j, const PointConnector &subject) {
  convert(j["limits"], {-1.f, -1.f}, {1.f, 1.f});
  j["boxes"] = subject.getBoxes();
}

void to_json(nlohmann::json &j, const Tree &subject) {
  j = nlohmann::json::array();
  for (const auto &node : subject) {
    const auto *father = node->getFatherInfo().father;
    convert(j.emplace_back(),
            (nullptr == father) ? node->getState() : father->getState(),
            node->getState());
  }
}

void to_json(nlohmann::json &j, const std::vector<Tree> &subject) {
  j = nlohmann::json::array();
  for (const auto &tree : subject) {
    to_json(j.emplace_back(), tree);
  }
}

void to_json(nlohmann::json &j, const ProblemDescription &problem,
             const PlannerSolution &solution,
             const ConnectorLogger &connector_logger,
             const SolutionLogger &solution_logger) {
  j["time_ms"] = solution.time.count();
  j["iterations"] = solution.iterations;

  connector_logger(j["scene"], *problem.connector);

  to_json(j["trees"], solution.trees);

  auto &solutions = j["solutions"];
  solutions = nlohmann::json::array();
  if (solution.solution) {
    auto &sol = solutions.emplace_back();
    sol["cost"] = 1.f;
    solution_logger(sol["sequence"], solution.solution.value());
  }
}

void to_json(nlohmann::json &j, const Extender &subject,
             const ConnectorLogger &connector_logger,
             const SolutionLogger &solution_logger) {
  connector_logger(j["scene"], *subject.problem().connector);

  to_json(j["trees"], subject.dumpTrees());

  auto &solutions = j["solutions"];
  solutions = nlohmann::json::array();
  for (const auto &[cost, solution] : subject.getSolutions()) {
    auto &sol = solutions.emplace_back();
    sol["cost"] = cost;
    solution_logger(sol["sequence"], solution->getSequence());
  }
}
} // namespace mt_rrt::utils
