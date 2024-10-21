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
    recipient.type = src["type"];
  }
  if (src.contains("threads")) {
    recipient.threads = src["threads"];
  }
  if (src.contains("synchronization")) {
    recipient.synchronization = src["synchronization"];
  }
}

std::unique_ptr<Planner> makePlanner(ProblemDescription &&desc, PlannerKind kind, const PlannerParameters &params) {
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
#ifdef MT_PLANNERS_ENABLED
  if (params.threads != 0) {
    auto *maybe_mt = dynamic_cast<MultiThreadedPlanner *>(res.get());
    if (maybe_mt) {
      maybe_mt->setThreads(params.threads);
    }
  }
  if (params.synchronization != 0) {
    auto *maybe_sy = dynamic_cast<SynchronizationAware *>(res.get());
    if (maybe_sy) {
      maybe_sy->synchronization().set(params.synchronization);
    }
  }
#endif  
  return res;
}

} // namespace mt_rrt
