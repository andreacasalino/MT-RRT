/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <LogResult.h>
#include <MT-RRT/Planner.h>
#include <MiscConversions.h>

#include <filesystem>
#include <fstream>
#include <iostream>

namespace mt_rrt {
void from_json(Parameters &recipient, const nlohmann::json &src);

template <typename ConnectorT>
ProblemDescription from_json(const nlohmann::json &src);

enum class PlannerKind {
  Standard
#ifdef MT_PLANNERS_ENABLED
  , Embarass
  , PQuery
  , Shared
  , Linked
  , Multiag
#endif
};

PlannerKind from_string(const std::string& str);

struct PlannerParameters {
  PlannerKind type = PlannerKind::Standard;
  std::size_t threads = 0;
  float synchronization = 0;
};

void from_json(PlannerParameters &type, const nlohmann::json &src);

std::unique_ptr<Planner> makePlanner(ProblemDescription &&desc, PlannerKind kind);

struct DefaultStateParser {
  std::vector<float> operator()(std::vector<float> subject) const {
    return subject;
  }
};

template <typename ConnectorT> class SampleBase {
public:
  SampleBase(const std::filesystem::path& src, const std::string& tag);
  SampleBase(int argc, const char **argv, const std::string &tag);

  template <typename StateParser = DefaultStateParser> void process();

private:
  static std::filesystem::path parse_source(int argc, const char** argv);

  std::string tag;

  nlohmann::json importData() {
    std::ifstream stream{source};
    return nlohmann::json::parse(stream);
  }

  std::filesystem::path source;
};

/////////////////////////////////////////////////////////////////////////////////////
template <typename ConnectorT>
SampleBase<ConnectorT>::SampleBase(const std::filesystem::path& src, const std::string& tag)
    : tag{tag}, source{src}  {
}

template <typename ConnectorT>
std::filesystem::path SampleBase<ConnectorT>::parse_source(int argc, const char** argv) {
    if (argc == 1) {
        throw Error{
            "Json file with the problem(s) description(s) was not specified" };
    }
    std::string res{ argv[1] };
#if _WIN64 || _WIN32
    // remove ' at the begin and at the end
    res = std::string{res, 1, res.size() - 2};
#endif
    return std::filesystem::path{ res };
}

template <typename ConnectorT>
SampleBase<ConnectorT>::SampleBase(int argc, const char **argv,
                                   const std::string &tag)
    : SampleBase{ parse_source(argc, argv), tag} {
}

template <typename ConnectorT>
template <typename StateParser>
void SampleBase<ConnectorT>::process() {
  StateParser state_parser;

  auto data = importData();
  Parameters globalParameters;
  from_json(globalParameters, data["parameters"]);
  PlannerParameters globalPlannerParameters;
  from_json(globalPlannerParameters, data["planner"]);
  auto the_problem = from_json<ConnectorT>(data["scene"]);
  auto planner =
      makePlanner(std::move(the_problem), globalPlannerParameters.type);

#ifdef MT_PLANNERS_ENABLED
  auto setUpPlanner = [&planner](const PlannerParameters& params) {
    if (params.threads != 0) {
      auto *maybe_mt = dynamic_cast<MultiThreadedPlanner *>(planner.get());
      if (maybe_mt) {
        maybe_mt->setThreads(params.threads);
      }
    }
    if (params.synchronization != 0) {
      auto *maybe_sy = dynamic_cast<SynchronizationAware *>(planner.get());
      if (maybe_sy) {
        maybe_sy->synchronization().set(params.synchronization);
      }
    }
  };
#endif  

  for (const auto &scenario : data["cases"]) {
    std::string title = scenario["title"];
    std::cout << "----------------------------------------------------------"
              << std::endl;
    std::cout << "solving: " << title << std::endl;
    Parameters parameters = globalParameters;
    if (scenario.contains("parameters")) {
      from_json(parameters, scenario["parameters"]);
    }
    PlannerParameters plannerParameters = globalPlannerParameters;
    if (scenario.contains("planner")) {
      from_json(plannerParameters, scenario["planner"]);
    }
    std::vector<float> start = state_parser(scenario["start"]);
    std::vector<float> end = state_parser(scenario["end"]);
#ifdef MT_PLANNERS_ENABLED
    if (scenario.contains("parameters")) {
      setUpPlanner(plannerParameters);
    }
#endif  
    auto solution = planner->solve(start, end, parameters);
    if (solution.solution.empty()) {
      std::cout << "A solution was NOT found" << std::endl;
    } else {
      std::cout << "A solution was found in " << solution.time.count()
                << " [us] using " << solution.iterations << " iterations"
                << std::endl;
    }
    LogResult result;
    to_json(result,
            static_cast<const ConnectorT &>(*planner->problem().connector));
    result.addToScene("start") = start;
    result.addToScene("end") = end;

    solution.solution = extract_solution<ConnectorT>(
        static_cast<const ConnectorT &>(*planner->problem().connector),
        solution.solution);
    result.addPlannerSolution(solution);

    Logger::get().add(tag, title, result.get());
  }
}

} // namespace mt_rrt
