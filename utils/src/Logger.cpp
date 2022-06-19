/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT-carpet/Strings.h>

#include <JsonConvert.h>
#include <Logger.h>

#include <filesystem>
#include <fstream>
#include <iostream>

namespace mt_rrt::utils {
std::unordered_map<std::string, std::size_t> Logger::counters =
    std::unordered_map<std::string, std::size_t>{};

void Logger::log(const std::string &tag, const nlohmann::json &content,
                 const std::optional<std::string> &python_script) {
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

  if (python_script) {
    std::filesystem::path python_script_location = python_script.value();

    const auto script_name = python_script_location.filename();
    std::filesystem::path python_script_destination =
        merge(folder_name, '/', script_name.c_str());

    if (!std::filesystem::exists(python_script_destination)) {
      std::filesystem::copy_file(python_script_location,
                                 python_script_destination);
    }

    std::cout << "run `python3 " << python_script_destination.c_str() << ' '
              << log_name << '`' << std::endl;
  }
}

void log_scenario(
    const ProblemDescription &problem, const PlannerSolution &solution,
    const ConnectorLogger &connector_logger,
    const SolutionLogger &solution_logger, const std::string &case_name,
    const std::optional<std::string> &python_visualization_script) {
  nlohmann::json json_log;
  to_json(json_log, problem, solution, connector_logger, solution_logger);
  Logger::log(merge("extender-", case_name), json_log,
              python_visualization_script);
}

void log_scenario(
    const mt_rrt::Extender &subject, const ConnectorLogger &connector_logger,
    const SolutionLogger &solution_logger, const std::string &case_name,
    const std::optional<std::string> &python_visualization_script) {
  nlohmann::json json_log;
  to_json(json_log, subject, connector_logger, solution_logger);
  Logger::log(merge("extender-", case_name), json_log,
              python_visualization_script);
}
} // namespace mt_rrt::utils
