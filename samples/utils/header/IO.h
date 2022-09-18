/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/../../src/Extender.h>
#include <MT-RRT-core/Planner.h>

#include <nlohmann/json.hpp>

#include <unordered_map>

namespace mt_rrt::utils {
void from_file(nlohmann::json &j, const std::string &fileName);

// needs to be implemented in a different way for the various samples
class Converter {
public:
  virtual ~Converter() = default;

  virtual void fromJson(const nlohmann::json &json,
                        ProblemDescription &description) const = 0;
  virtual void toJson(nlohmann::json &json,
                      const ProblemDescription &description) const = 0;

  virtual std::vector<State>
      interpolate(const ProblemDescription& description,
          const std::vector<State>& solution) const {
      return solution;
  };

  void toJson2(nlohmann::json& recipient, const ProblemDescription& problem, const PlannerSolution& solution) const;
  void toJson2(nlohmann::json& recipient, const mt_rrt::Extender& extender) const;
};

class PythonSources {
public:
  PythonSources(const std::string &source) { sources.push_back(source); }

  PythonSources(const std::vector<std::string> &sources);

  // assembles and write all sources together in a single python file
  void reprint(const std::string &destination) const;

private:
  std::vector<std::string> sources;
};

PythonSources default_python_sources(const std::string &problem_script);

class Logger {
public:
    struct Log {
        std::string tag;
        nlohmann::json content;
        std::optional<PythonSources> python_visualizer;
    };
  static void log(const Log& to_log);

private:
    static std::unordered_map<std::string, std::size_t> counters; // <Log::tag, entities_with_that_tag>
};
} // namespace mt_rrt::utils
