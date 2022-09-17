/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/Parameters.h>
#include <MT-RRT-core/Planner.h>

#include <iostream>
#include <nlohmann/json.hpp>

namespace mt_rrt::utils {
using Problems = std::vector<std::pair<State, State>>;

class SampleFramework {
public:

  // expects 1 single arguments which can be (auto deduced internally):
  //  - the file path of the json with the additional configurations
  //  - a string representing the json with the additional configurations
  SampleFramework(int argc, const char **argv);

  void init();

  void show(std::ostream& recipient) const;

  Planner& planner() const { return *planner_; }

  const Problems& problems() const { return problems_; };

  void log(PlannerSolution solution, const std::string& tag, const std::string& python_script) const;

protected:
  virtual std::string defaultJsonConfig() const = 0;
  virtual void fromJsonProblemDescription(const nlohmann::json& json, ProblemDescription& description) = 0;
  virtual void toJsonProblemDescription(nlohmann::json& json, const ProblemDescription& description) = 0;

  std::unique_ptr<Planner> planner_;
  Problems problems_;
};
} // namespace mt_rrt::samples

