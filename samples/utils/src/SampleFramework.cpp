#include <MT-RRT-carpet/Error.h>

#include <MT-RRT-core/StandardPlanner.h>
#include <MT-RRT-multi-threaded/EmbarassinglyParallel.h>
#include <MT-RRT-multi-threaded/LinkedTreesPlanner.h>
#include <MT-RRT-multi-threaded/MultiAgentPlanner.h>
#include <MT-RRT-multi-threaded/ParallelizedQueriesPlanner.h>
#include <MT-RRT-multi-threaded/SharedTreePlanner.h>

#include <JsonConvert.h>
#include <SampleFramework.h>

#include <filesystem>
#include <iostream>

namespace mt_rrt::utils {
SampleFramework::SampleFramework(int argc, const char **argv) {
  if (argc > 2) {
    throw Error{"Invalid arguments"};
  }
  argument_.emplace(argv[1]);
}

namespace {
template <typename Alternative> void try_alternative(Alternative &&cb) {
  try {
    cb();
  } catch (...) {
  }
}

template <typename Alternative, typename... Others>
void try_alternative(Alternative &&cb, Others... others) {
  try {
    cb();
  } catch (...) {
    try_alternative(std::forward<Others>(others)...);
  }
}

void merge_overriding(nlohmann::json &recipient,
                      const nlohmann::json &to_merge) {
  if (!to_merge.is_structured()) {
    return;
  }
  for (auto &[key, val] : to_merge.items()) {
    recipient[key] = val;
  }
}

void from_json(const nlohmann::json &json, Parameters &recipient) {
  recipient.expansion_strategy = ExpansionStrategy::Star;
  recipient.steer_trials.set(10);
  recipient.iterations.set(2500);
  recipient.determinism.set(0.15f);
  recipient.best_effort = true;

  if (json.contains("params")) {
    auto &j_par = json["params"];

    if (j_par.contains("strategy")) {
      std::string strategy = j_par.at("strategy");
      if (strategy == "Single") {
        recipient.expansion_strategy = ExpansionStrategy::Single;
      } else if (strategy == "Bidir") {
        recipient.expansion_strategy = ExpansionStrategy::Bidir;
      } else if (strategy == "Star") {
        recipient.expansion_strategy = ExpansionStrategy::Star;
      }
    }

    if (j_par.contains("steer_trials")) {
      recipient.steer_trials.set(j_par.at("steer_trials"));
    }

    if (j_par.contains("iterations")) {
      recipient.iterations.set(j_par.at("iterations"));
    }

    if (j_par.contains("determinism")) {
      recipient.determinism.set(j_par.at("determinism"));
    }

    if (j_par.contains("best_effort")) {
      recipient.best_effort = j_par.at("best_effort");
    }
  }
}

void from_json(const nlohmann::json &json, std::pair<State, State> &recipient) {
  auto &[start, end] = recipient;
  nlohmann::from_json(json["start"], start);
  nlohmann::from_json(json["end"], end);
}

void set_threads(MultiThreadedPlanner &planner, const nlohmann::json &j) {
  if (j.contains("threads")) {
    std::size_t threads = j.at("threads");
    if (threads <= 1) {
      planner.setMaxThreads();
    } else {
      planner.setThreads(Threads{threads});
    }
  }
}

void set_synchronization(SynchronizationAware &planner,
                         const nlohmann::json &j) {
  if (j.contains("synch")) {
    planner.synchronization().set(j.at("synch"));
  }
}

std::unique_ptr<Planner> planner_from_json(const nlohmann::json &json,
                                           ProblemDescription &&description) {
  if (json.contains("planner")) {
    const auto &j_planner = json["planner"];

    std::string planner_type;
    if (j_planner.contains("type")) {
      planner_type = j_planner.at("type");
    }

    if (planner_type == "embarassingly") {
      auto planner = std::make_unique<EmbarassinglyParallelPlanner>(
          std::forward<ProblemDescription>(description));
      set_threads(*planner, j_planner);
      return planner;
    }
    if (planner_type == "parall_query") {
      auto planner = std::make_unique<ParallelizedQueriesPlanner>(
          std::forward<ProblemDescription>(description));
      set_threads(*planner, j_planner);
      return planner;
    }
    if (planner_type == "shared") {
      auto planner = std::make_unique<SharedTreePlanner>(
          std::forward<ProblemDescription>(description));
      set_threads(*planner, j_planner);
      return planner;
    }
    if (planner_type == "linked") {
      auto planner = std::make_unique<LinkedTreesPlanner>(
          std::forward<ProblemDescription>(description));
      set_threads(*planner, j_planner);
      set_synchronization(*planner, j_planner);
      return planner;
    }
    if (planner_type == "agents") {
      auto planner = std::make_unique<MultiAgentPlanner>(
          std::forward<ProblemDescription>(description));
      set_threads(*planner, j_planner);
      set_synchronization(*planner, j_planner);
      return planner;
    }
  }

  return std::make_unique<StandardPlanner>(
      std::forward<ProblemDescription>(description));
}
} // namespace

void SampleFramework::init() {
  from_file(config_, defaultJsonConfig());
  if (argument_) {
    try_alternative(
        [&]() {
          nlohmann::json config_override;
          from_file(config_override, this->argument_.value());
          merge_overriding(this->config_, config_override);
        },
        [&]() {
          nlohmann::json config_override =
              nlohmann::json::parse(this->argument_.value());
          merge_overriding(this->config_, config_override);
        });
  }

  from_json(config_["Parameters"], parameters_);

  nlohmann::from_json(config_["Problems"], problems_);

  ProblemDescription description;
  fromJson(config_["ProblemDescription"], description);
  planner_ = planner_from_json(config_["Planner"], std::move(description));
}

namespace {
static const std::string FORMAT_SPACE_DELTA = "  ";

void printConfigurations(std::ostream &recipient, std::string format_space,
                         const nlohmann::json &content) {
  if (content.is_structured()) {
    recipient << '{' << std::endl;
    for (const auto &[key, val] : content.items()) {
      recipient << format_space << '\"' << key << "\":";
      printConfigurations(recipient, format_space + FORMAT_SPACE_DELTA, val);
    }
    recipient << format_space << '}' << std::endl;
    return;
  }
  if (content.is_array()) {
    recipient << format_space << '[' << std::endl;
    for (const auto &element : content) {
      recipient << format_space;
      printConfigurations(recipient, format_space + FORMAT_SPACE_DELTA,
                          element);
    }
    recipient << format_space << ']' << std::endl;
    return;
  }
  recipient << content.dump() << std::endl;
}
} // namespace

void SampleFramework::show(std::ostream &recipient) const {
  printConfigurations(recipient, "", config_);
  recipient << "===================================================="
            << std::endl
            << std::endl;
}
} // namespace mt_rrt::utils

std::ostream &operator<<(std::ostream &s, const mt_rrt::State &subject) {
  for (const auto val : subject) {
    s << ' ' << val;
  }
  return s;
}

std::ostream &operator<<(std::ostream &s,
                         const mt_rrt::utils::SampleFramework &subject) {
  subject.show(s);
  return s;
}
