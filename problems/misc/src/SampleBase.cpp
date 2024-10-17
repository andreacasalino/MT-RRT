#include <SampleBase.h>

#include <MT-RRT/StandardPlanner.h>
#ifdef MT_PLANNERS_ENABLED
#include <MT-RRT/EmbarassinglyParallel.h>
#include <MT-RRT/LinkedTreesPlanner.h>
#include <MT-RRT/MultiAgentPlanner.h>
#include <MT-RRT/ParallelizedQueriesPlanner.h>
#include <MT-RRT/SharedTreePlanner.h>
#endif

#include <unordered_map>

namespace mt_rrt {
PlannerKind from_string(const std::string& str) {
  PlannerKind res;
  bool found = false;

  if(str == "STANDARD") {
    res = PlannerKind::Standard;
    found = true;
  }

#ifdef MT_PLANNERS_ENABLED
  if(str == "EMBARASS") {
    res = PlannerKind::Embarass;
    found = true;
  }
  if(str == "PQUERY") {
    res = PlannerKind::PQuery;
    found = true;
  }
  if(str == "SHARED") {
    res = PlannerKind::Shared;
    found = true;
  }
  if(str == "LINKED") {
    res = PlannerKind::Linked;
    found = true;
  }
  if(str == "MULTIAG") {
    res = PlannerKind::Multiag;
    found = true;
  }
#endif

  if(!found) {
    throw std::runtime_error{"Unrecognized planner kind"};
  }

  return res;
}

void from_json(Parameters &recipient, const nlohmann::json &src) {
  if (src.contains("strategy")) {
    static std::unordered_map<std::string, ExpansionStrategy> strategies =
        std::unordered_map<std::string, ExpansionStrategy>{
            {"Single", ExpansionStrategy::Single},
            {"Star", ExpansionStrategy::Star},
            {"Bidir", ExpansionStrategy::Bidir}};
    std::string t = src["strategy"];
    auto it = strategies.find(t);
    if (it == strategies.end()) {
      throw Error{t, " is an invalid strategy"};
    }
    recipient.expansion_strategy = it->second;
  }
  if (src.contains("steer_trials")) {
    recipient.steer_trials.set(src["steer_trials"]);
  }
  if (src.contains("iterations")) {
    recipient.iterations.set(src["iterations"]);
  }
  if (src.contains("determinism")) {
    recipient.determinism.set(src["determinism"]);
  }
  if (src.contains("best_effort")) {
    recipient.best_effort = src["best_effort"];
  }
}

void from_json(PlannerParameters &recipient, const nlohmann::json &src) {
  if (src.contains("type")) {
    recipient.type = from_string(src["type"]);
  }
  if (src.contains("threads")) {
    recipient.threads = src["threads"];
  }
  if (src.contains("synchronization")) {
    recipient.synchronization = src["synchronization"];
  }
}

std::unique_ptr<Planner> makePlanner(ProblemDescription &&desc, PlannerKind kind) {
  std::unique_ptr<Planner> res;
  switch (kind) {
  case PlannerKind::Standard:
    res = std::make_unique<StandardPlanner>(std::forward<ProblemDescription>(desc));
    break;
#ifdef MT_PLANNERS_ENABLED
  case PlannerKind::Embarass:
    res = std::make_unique<EmbarassinglyParallelPlanner>(std::forward<ProblemDescription>(desc));
    break;
  case PlannerKind::PQuery:
    res = std::make_unique<ParallelizedQueriesPlanner>(std::forward<ProblemDescription>(desc));
    break;
  case PlannerKind::Shared:
    res = std::make_unique<SharedTreePlanner>(std::forward<ProblemDescription>(desc));
    break;
  case PlannerKind::Linked:
    res = std::make_unique<LinkedTreesPlanner>(std::forward<ProblemDescription>(desc));
    break;
  case PlannerKind::Multiag:
    res = std::make_unique<MultiAgentPlanner>(std::forward<ProblemDescription>(desc));
    break;
#endif  
  }
  return res;
}

} // namespace mt_rrt
