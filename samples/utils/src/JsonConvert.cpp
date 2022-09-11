/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <JsonConvert.h>

#include <fstream>

namespace mt_rrt::utils {
void from_file(nlohmann::json &j, const std::string &fileName) {
  std::stringstream buffer;
  {
    std::ifstream stream(fileName);
    if (!stream.is_open()) {
      throw Error{fileName, " is not a valid.json file"};
    }
    buffer << stream.rdbuf();
  }
  j = nlohmann::json::parse(buffer.str());
}

namespace {
void convert(nlohmann::json &j, const State &start, const State &end) {
  j["start"] = start;
  j["end"] = end;
}
} // namespace

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
             const PlannerSolution &solution, const Converter &converter) {
  j["time_ms"] = solution.time.count();
  j["iterations"] = solution.iterations;

  converter.toJson(j["scene"], *problem.connector);

  to_json(j["trees"], solution.trees);

  auto &solutions = j["solutions"];
  solutions = nlohmann::json::array();
  if (solution.solution) {
    auto &sol = solutions.emplace_back();
    sol["cost"] = 1.f;
    converter.toJson(sol["sequence"], solution.solution.value(),
                     *problem.connector);
  }
}

void to_json(nlohmann::json &j, const Extender &subject,
             const Converter &converter) {
  converter.toJson(j["scene"], *subject.problem().connector);

  to_json(j["trees"], subject.dumpTrees());

  auto &solutions = j["solutions"];
  solutions = nlohmann::json::array();
  for (const auto &[cost, solution] : subject.getSolutions()) {
    auto &sol = solutions.emplace_back();
    sol["cost"] = cost;
    converter.toJson(sol["sequence"], solution->getSequence(),
                     *subject.problem().connector);
  }
}
} // namespace mt_rrt::utils
