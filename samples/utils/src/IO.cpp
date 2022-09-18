/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <IO.h>

#include <filesystem>
#include <fstream>
#include <iostream>

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

void to_json(nlohmann::json &j, const Tree &subject) {
  j = nlohmann::json::array();
  for (const auto &node : subject) {
    const auto *father = node->getFatherInfo().father;
    convert(j.emplace_back(),
            (nullptr == father) ? node->getState() : father->getState(),
            node->getState());
  }
}
} // namespace

void Converter::toJson2(nlohmann::json& j, const ProblemDescription& problem, const PlannerSolution& solution) const {
    j["time_ms"] = solution.time.count();
    j["iterations"] = solution.iterations;

    toJson(j["scene"], problem);

    auto& trees = j["trees"];
    trees = nlohmann::json::array();
    for (const auto& tree : solution.trees) {
        auto& tree_json = trees.emplace_back();
        to_json(tree_json, tree);
    }

    auto& solutions = j["solutions"];
    solutions = nlohmann::json::array();
    if (solution.solution) {
        auto& sol = solutions.emplace_back();
        sol["cost"] = 1.f;
        sol["sequence"] = interpolate(problem, solution.solution.value());
    }
}

void Converter::toJson2(nlohmann::json& j, const mt_rrt::Extender& extender) const {
    const auto& problem = extender.problem();
    toJson(j["scene"], problem);

    auto& trees = j["trees"];
    trees = nlohmann::json::array();
    for (const auto& tree : extender.dumpTrees()) {
        auto& tree_json = trees.emplace_back();
        to_json(tree_json, tree);
    }

    auto& solutions = j["solutions"];
    solutions = nlohmann::json::array();
    for (const auto& [cost, solution] : extender.getSolutions()) {
        auto& sol = solutions.emplace_back();
        sol["cost"] = cost;
        sol["sequence"] = interpolate(problem, solution->getSequence());
    }
}

PythonSources::PythonSources(const std::vector<std::string> &filesNames)
    : sources(filesNames) {}

void PythonSources::reprint(const std::string &destination) const {
  std::ofstream out_stream(destination);
  for (const auto &path : sources) {
    std::ifstream in_stream(path);
    if (!in_stream.is_open()) {
      throw Error{path, " is an invalid source"};
    }
    out_stream << std::endl << in_stream.rdbuf();
  }
}

PythonSources default_python_sources(const std::string &problem_script) {
  return PythonSources{std::vector<std::string>{PYTHON_SHOW_PRE, problem_script,
                                                PYTHON_SHOW_POST}};
}

std::unordered_map<std::string, std::size_t> Logger::counters =
    std::unordered_map<std::string, std::size_t>{};

void Logger::log(const Log& to_log) {
  const auto& [tag, content, python] = to_log;

  const auto folder_name = merge("log-", tag);
  auto counter_it = counters.find(folder_name);
  if (counter_it == counters.end()) {
    counter_it = counters.emplace(folder_name, 0).first;
    std::filesystem::remove_all(folder_name);
    std::filesystem::create_directory(folder_name);
  }
  const auto log_name =
      merge(folder_name, "/log-", std::to_string(counter_it->second), ".json");
  ++counter_it->second;

  std::ofstream{log_name} << content.dump();

  if (python) {
    std::string python_script_destination = merge(folder_name, "/Show.py");

    python->reprint(python_script_destination);

    std::cout << "run `python3 " << python_script_destination.c_str() << ' '
              << log_name << '`' << std::endl;
  }
}
} // namespace mt_rrt::utils
