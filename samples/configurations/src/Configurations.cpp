#include <MT-RRT-carpet/Error.h>

#include <MT-RRT-core/StandardPlanner.h>
#include <MT-RRT-multi-threaded/EmbarassinglyParallel.h>
#include <MT-RRT-multi-threaded/LinkedTreesPlanner.h>
#include <MT-RRT-multi-threaded/MultiAgentPlanner.h>
#include <MT-RRT-multi-threaded/ParallelizedQueriesPlanner.h>
#include <MT-RRT-multi-threaded/SharedTreePlanner.h>

#include <Configurations.h>
#include <JsonConvert.h>

#include <iostream>

namespace mt_rrt::samples {
namespace {
void throw_bad_syntax() { throw Error{"Invalid arguments\n", HELP}; }
} // namespace

void to_json(nlohmann::json &configurations, int argc, const char **argv) {
  std::cout << "=================================================="
            << std::endl;
  if (argc <= 1) {
    return;
  }

  const std::string argv_1 = argv[1];
  if ((argv_1 == "-h") || (argv_1 == "--help")) {
    throw Error{HELP};
  }

  if (argc != 3) {
    throw_bad_syntax();
  }
  const std::string argv_2 = argv[2];

  if (argv_1 == "-f") {
    std::cout << "reading configurations from " << argv_2 << std::endl;
    utils::from_file(configurations, argv_2);
  } else if (argv_1 == "-j") {
    configurations = nlohmann::json::parse(argv_2);
  } else if ((argv_1 == "-h") || (argv_1 == "-help")) {
    configurations = nlohmann::json::parse(argv_2);
  } else {
    throw_bad_syntax();
  }
  std::cout << configurations << std::endl;
  std::cout << "==================================================" << std::endl
            << std::endl;
}

void from_json(const nlohmann::json &j, Parameters &recipient) {
  if (!j.contains("params")) {
    return;
  }
  auto &j_par = j["params"];

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

namespace {
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
} // namespace

std::unique_ptr<Planner> from_json(const nlohmann::json &j,
                                   ProblemDescription &&problem) {
  if (j.contains("planner")) {
    const auto &j_planner = j.at("planner");

    std::string planner_type;
    if (j_planner.contains("type")) {
      planner_type = j_planner.at("type");
    }

    if (planner_type == "embarassingly") {
      auto planner = std::make_unique<EmbarassinglyParallelPlanner>(
          std::forward<ProblemDescription>(problem));
      set_threads(*planner, j_planner);
      return planner;
    }
    if (planner_type == "parall_query") {
      auto planner = std::make_unique<ParallelizedQueriesPlanner>(
          std::forward<ProblemDescription>(problem));
      set_threads(*planner, j_planner);
      return planner;
    }
    if (planner_type == "shared") {
      auto planner = std::make_unique<SharedTreePlanner>(
          std::forward<ProblemDescription>(problem));
      set_threads(*planner, j_planner);
      return planner;
    }
    if (planner_type == "linked") {
      auto planner = std::make_unique<LinkedTreesPlanner>(
          std::forward<ProblemDescription>(problem));
      set_threads(*planner, j_planner);
      set_synchronization(*planner, j_planner);
      return planner;
    }
    if (planner_type == "agents") {
      auto planner = std::make_unique<MultiAgentPlanner>(
          std::forward<ProblemDescription>(problem));
      set_threads(*planner, j_planner);
      set_synchronization(*planner, j_planner);
      return planner;
    }
  }

  return std::make_unique<StandardPlanner>(
      std::forward<ProblemDescription>(problem));
}
} // namespace mt_rrt::samples
