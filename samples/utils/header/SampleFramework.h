/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/Parameters.h>
#include <MT-RRT-core/Planner.h>

#include <JsonConvert.h>

#include <iostream>

namespace mt_rrt::utils {
using Problems = std::vector<std::pair<State, State>>;

class SampleFramework : public ProblemDescriptionConverter {
public:
  // expects 1 single argument which can be (auto deduced internally):
  //  - the file path of the json with the additional configurations
  //  - a string representing the json with the additional configurations
  SampleFramework(int argc, const char **argv);

  void init();

  void show(std::ostream &recipient) const;

  Planner &planner() const { return *planner_; }

  const Problems &problems() const { return problems_; };
  const Parameters &parameters() const { return parameters_; }
  const ProblemDescription &problemDescription() const {
    return planner_->problem();
  }

protected:
  virtual std::string defaultJsonConfig() const = 0;

  std::optional<std::string> argument_;
  nlohmann::json config_;

  Parameters parameters_;
  std::unique_ptr<Planner> planner_;
  Problems problems_;
};
} // namespace mt_rrt::utils

std::ostream &operator<<(std::ostream &s, const mt_rrt::State &subject);

std::ostream &operator<<(std::ostream &s,
                         const mt_rrt::utils::SampleFramework &subject);
