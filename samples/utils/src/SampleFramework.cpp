//#include <MT-RRT-carpet/Error.h>
//
//#include <MT-RRT-core/StandardPlanner.h>
//#include <MT-RRT-multi-threaded/EmbarassinglyParallel.h>
//#include <MT-RRT-multi-threaded/LinkedTreesPlanner.h>
//#include <MT-RRT-multi-threaded/MultiAgentPlanner.h>
//#include <MT-RRT-multi-threaded/ParallelizedQueriesPlanner.h>
//#include <MT-RRT-multi-threaded/SharedTreePlanner.h>
//
//#include <JsonConvert.h>
//#include <SampleFramework.h>
//
//#include <filesystem>
//#include <iostream>
//
//namespace mt_rrt::utils {
//namespace {
//void merge_overriding(nlohmann::json &recipient,
//                      const nlohmann::json &to_merge) {
//  if (!to_merge.is_structured()) {
//    return;
//  }
//  for (auto &[key, val] : to_merge.items()) {
//    recipient[key] = val;
//  }
//}
//} // namespace
//
//SampleFramework::SampleFramework(const std::string &default_config_json) {
//  utils::from_file(configurations, default_config_json);
//  if (!configurations.contains("seed")) {
//    configurations["seed"] = 0;
//  }
//}
//
//namespace {
//void parse_user_json(nlohmann::json &recipient, const std::string &argv) {
//  if (std::filesystem::exists(argv)) {
//    utils::from_file(recipient, argv);
//    return;
//  }
//
//  recipient = nlohmann::json::parse(argv);
//}
//} // namespace
//
//SampleFramework::SampleFramework(const std::string &default_config_json,
//                                 int argc, const char **argv)
//    : SampleFramework{default_config_json} {
//  if (argc <= 1) {
//    return;
//  }
//  if (argc > 2) {
//    throw Error{"Invalid arguments"};
//  }
//
//  nlohmann::json user_json;
//  try {
//    parse_user_json(user_json, argv[1]);
//  } catch (...) {
//    throw Error{"Invalid arguments"};
//  }
//
//  if (user_json.contains("cases")) {
//    configurations["cases"] = user_json["cases"];
//  }
//  if (user_json.contains("seed")) {
//    configurations["seed"] = user_json["seed"];
//  }
//  auto merger = [&](const std::string &field) {
//    if (user_json.contains(field)) {
//      merge_overriding(configurations[field], user_json[field]);
//    }
//  };
//  merger("params");
//  merger("planner");
//}
//
//Parameters SampleFramework::getParameters() {
//  Parameters result;
//
//  result.expansion_strategy = ExpansionStrategy::Star;
//  result.steer_trials.set(10);
//  result.iterations.set(2500);
//  result.determinism.set(0.15f);
//  result.best_effort = true;
//
//  if (configurations.contains("params")) {
//    auto &j_par = configurations["params"];
//
//    if (j_par.contains("strategy")) {
//      std::string strategy = j_par.at("strategy");
//      if (strategy == "Single") {
//        result.expansion_strategy = ExpansionStrategy::Single;
//      } else if (strategy == "Bidir") {
//        result.expansion_strategy = ExpansionStrategy::Bidir;
//      } else if (strategy == "Star") {
//        result.expansion_strategy = ExpansionStrategy::Star;
//      }
//    }
//
//    if (j_par.contains("steer_trials")) {
//      result.steer_trials.set(j_par.at("steer_trials"));
//    }
//
//    if (j_par.contains("iterations")) {
//      result.iterations.set(j_par.at("iterations"));
//    }
//
//    if (j_par.contains("determinism")) {
//      result.determinism.set(j_par.at("determinism"));
//    }
//
//    if (j_par.contains("best_effort")) {
//      result.best_effort = j_par.at("best_effort");
//    }
//  }
//
//  return result;
//}
//
//namespace {
//void set_threads(MultiThreadedPlanner &planner, const nlohmann::json &j) {
//  if (j.contains("threads")) {
//    std::size_t threads = j.at("threads");
//    if (threads <= 1) {
//      planner.setMaxThreads();
//    } else {
//      planner.setThreads(Threads{threads});
//    }
//  }
//}
//
//void set_synchronization(SynchronizationAware &planner,
//                         const nlohmann::json &j) {
//  if (j.contains("synch")) {
//    planner.synchronization().set(j.at("synch"));
//  }
//}
//} // namespace
//
//std::unique_ptr<Planner>
//SampleFramework::getPlanner(std::shared_ptr<ProblemDescription> description) {
//  if (configurations.contains("planner")) {
//    const auto &j_planner = configurations["planner"];
//
//    std::string planner_type;
//    if (j_planner.contains("type")) {
//      planner_type = j_planner.at("type");
//    }
//
//    if (planner_type == "embarassingly") {
//      auto planner =
//          std::make_unique<EmbarassinglyParallelPlanner>(description);
//      set_threads(*planner, j_planner);
//      return planner;
//    }
//    if (planner_type == "parall_query") {
//      auto planner = std::make_unique<ParallelizedQueriesPlanner>(description);
//      set_threads(*planner, j_planner);
//      return planner;
//    }
//    if (planner_type == "shared") {
//      auto planner = std::make_unique<SharedTreePlanner>(description);
//      set_threads(*planner, j_planner);
//      return planner;
//    }
//    if (planner_type == "linked") {
//      auto planner = std::make_unique<LinkedTreesPlanner>(description);
//      set_threads(*planner, j_planner);
//      set_synchronization(*planner, j_planner);
//      return planner;
//    }
//    if (planner_type == "agents") {
//      auto planner = std::make_unique<MultiAgentPlanner>(description);
//      set_threads(*planner, j_planner);
//      set_synchronization(*planner, j_planner);
//      return planner;
//    }
//  }
//
//  return std::make_unique<StandardPlanner>(description);
//}
//
//std::vector<std::pair<State, State>> SampleFramework::getCases() {
//  std::vector<std::pair<State, State>> result;
//
//  if (configurations.contains("cases")) {
//    auto &j_cases = configurations["cases"];
//    for (const auto &j_case : j_cases) {
//      State start = j_case["start"];
//      State end = j_case["end"];
//      auto &added = result.emplace_back(std::make_pair(start, end));
//      setCase(added);
//    }
//  }
//
//  return result;
//}
//
//namespace {
//static const std::string FORMAT_SPACE_DELTA = "  ";
//
//void printConfigurations(std::ostream &recipient, std::string format_space,
//                         const nlohmann::json &content) {
//  if (content.is_structured()) {
//    recipient << '{' << std::endl;
//    for (const auto &[key, val] : content.items()) {
//      recipient << format_space << '\"' << key << "\":";
//      printConfigurations(recipient, format_space + FORMAT_SPACE_DELTA, val);
//    }
//    recipient << format_space << '}' << std::endl;
//    return;
//  }
//  if (content.is_array()) {
//    recipient << format_space << '[' << std::endl;
//    for (const auto &element : content) {
//      recipient << format_space;
//      printConfigurations(recipient, format_space + FORMAT_SPACE_DELTA,
//                          element);
//    }
//    recipient << format_space << ']' << std::endl;
//    return;
//  }
//  recipient << content.dump() << std::endl;
//}
//} // namespace
//
//void SampleFramework::showConfigurations(std::ostream &recipient) const {
//  printConfigurations(recipient, "", configurations);
//  recipient << "===================================================="
//            << std::endl
//            << std::endl;
//}
//} // namespace mt_rrt::samples
//
//std::ostream &operator<<(std::ostream &s, const mt_rrt::State &subject) {
//  for (const auto val : subject) {
//    s << ' ' << val;
//  }
//  return s;
//}
//
//std::ostream &operator<<(std::ostream &s,
//                         const mt_rrt::samples::SampleFramework &subject) {
//  subject.showConfigurations(s);
//  return s;
//}
