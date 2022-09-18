/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/Parameters.h>
#include <MT-RRT-core/Planner.h>

#include <IO.h>

namespace mt_rrt::utils {
using Problems = std::vector<std::pair<State, State>>;

class SampleFramework {
public:
  // All configurations are red from a default .json file (whose location is
  // returned by defaultJsonConfig()), different from sample to sample.
  // Refer to the comment on top of defaultJsonConfig() for the format of such a
  // .json file.
  //
  // You can override any of the default settings by passing 1 single
  // argument, whose meaning is to be (auto deduced internally):
  //  - the file path of another .json with the overrides
  //  - a string representing the .json with the overrides
  SampleFramework(int argc, const char **argv, const Converter &converter);

  void init();

  void show(std::ostream &recipient) const;

  Planner &planner() const { return *planner_; }

  const Problems &problems() const { return problems_; };
  const Parameters &parameters() const { return parameters_; }
  const ProblemDescription &problemDescription() const {
    return planner_->problem();
  }

protected:
  // This is the format accepted for the default .json used to configure the sample, as well as for
  // the one the user may pass to override any or all of the configuration fields:
  //
  // config["Problems"] -> the array [., ..., .] of specific problems to solve. each element contains:
  // 	["start"] -> the start state
  // 	["end"] -> the ending state to connect with the start one
  //
  // config["Parameters"] -> the parameters considered when solving any specific problem
  // 	config["Parameters"]["strategy"] -> can be "Single", "Bidir", "Star" implying to use a single, bidirectional or star RRT approach
  //    config["Parameters"]["steer_trials"] -> steering trials
  //    config["Parameters"]["iterations"] -> maximum number of iterations used by a Planner to get the solution
  //    config["Parameters"]["determinism"] -> determinism degree (number in [0,1])
  //    config["Parameters"]["best_effort"] -> when set to true, as soon as a solution is found the search is arrested (unless a RRT* approach is used). When set to false, solutions are cumulated.
  //
  // config["Planner"] -> it can be:
  //							- "embarassingly", for prescribing the adoption of an EmbarassinglyParallel planner
  //							- "parall_query", for prescribing the adoption of an ParallelizedQueriesPlanner planner
  //							- "shared", for prescribing the adoption of an SharedTreePlanner planner
  //							- "linked", for prescribing the adoption of an LinkedTreesPlanner planner
  //							- "agents", for prescribing the adoption of an MultiAgentPlanner planner
  // in case this field is not specified, a StandardPlanner will be adopted
  // in case "embarassingly", "parall_query", "shared", "linked" or "agents" was specified, you can also specify the number of threads in a subfield:
  // 	config["Planner"]["threads"]
  // in case "linked" or "agents" was specified, you can also specify the synchronization degree in a subfield:
  //    config["Planner"]["synch"] (number in [0,1])
  //
  // config["ProblemDescription"] -> a field that is parsed in different ways according to the particular problem to solve. According to its content, 
  // a mt_rrt::ProblemDescription object is built in order to generate the Planner
  virtual std::string defaultJsonConfig() const = 0;

  const Converter &converter_;

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
