///**
// * Author:    Andrea Casalino
// * Created:   16.05.2019
// *
// * report any bug to andrecasa91@gmail.com.
// **/
//
//#pragma once
//
//#include <MT-RRT-core/Parameters.h>
//#include <MT-RRT-core/Planner.h>
//
//#include <iostream>
//#include <nlohmann/json.hpp>
//
//namespace mt_rrt::samples {
//class SampleFramework {
//public:
//  SampleFramework(const std::string &default_config_json);
//
//  // expects 1 single arguments which can be (auto deduced internally):
//  //  - the file path of the json with the additional configurations
//  //  - a string representing the json with the additional configurations
//  SampleFramework(const std::string &default_config_json, int argc,
//                  const char **argv);
//
//  std::shared_ptr<ProblemDescription> getProblemDescription() {
//    return getProblemDescription_(configurations["scene"]);
//  }
//
//  Parameters getParameters();
//
//  std::unique_ptr<Planner>
//  getPlanner(std::shared_ptr<ProblemDescription> description);
//
//  std::vector<std::pair<State, State>> getCases();
//
//  void showConfigurations(std::ostream &recipient) const;
//
//protected:
//  virtual std::shared_ptr<ProblemDescription>
//  getProblemDescription_(const nlohmann::json &scene_json) = 0;
//
//  std::optional<Seed> getSeed() {
//    std::optional<Seed> result;
//    result.emplace() = configurations["seed"];
//    return result;
//  }
//
//  virtual void setCase(std::pair<State, State> &start_end) { return; };
//
//private:
//  // The configuration json is expected to contain.
//  //
//  // - j["params"] -> parameters used to get solution to problem cases
//  //    - j["params"]["strategy"] -> a string that can be equal to:
//  //    "Single", "Bidir" or "Star"
//  //    - j["params"]["steer_trials"] -> number of steers
//  //    - j["params"]["iterations"] -> iterations
//  //    - j["params"]["determinism"] -> floating number in [0,1]
//  //    - j["params"]["best_effort] -> boolean
//  //
//  // - j["planner"] -> info used to build and setup solver
//  //    - j["planner"]["type"] -> a string that can be equal to:
//  //       - "embarassingly"
//  //       - "parall_query"
//  //       - "shared"
//  //       - "linked"
//  //       - "agents"
//  //       if not specified a standard (non multithreaded) planner is built
//  //
//  //    - j["planner"]["threads"] -> number of threads. Meaningful only in
//  //    case
//  //      j["planner"]["type"] was specified
//  //
//  //    - j["planner"]["synch"] -> number in [0,1] to specify the
//  //      synchronization degree. Meaningful only in case
//  //      j["planner"]["type"] was set to "linked" or "agents"
//  //
//  // - j["scene"] -> static data describing the problem. Content expected
//  //   depends on the particular class of problem
//  //
//  // - j["cases"] -> [c0, ..., c1] each c is a pair of states to connect
//  //    - ck["start"]
//  //    - ck["end"]
//  //
//  // - j["seed"] -> seed for random engines
//  nlohmann::json configurations;
//};
//} // namespace mt_rrt::samples
//
//std::ostream &operator<<(std::ostream &s, const mt_rrt::State &subject);
//
//std::ostream &operator<<(std::ostream &s,
//                         const mt_rrt::samples::SampleFramework &subject);
