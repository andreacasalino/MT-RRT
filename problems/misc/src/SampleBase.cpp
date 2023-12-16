#include <SampleBase.h>

#include <MT-RRT/StandardPlanner.h>

#include <MT-RRT/EmbarassinglyParallel.h>
#include <MT-RRT/LinkedTreesPlanner.h>
#include <MT-RRT/MultiAgentPlanner.h>
#include <MT-RRT/ParallelizedQueriesPlanner.h>
#include <MT-RRT/SharedTreePlanner.h>

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

namespace {
class PlannerFactory {
public:
  std::unique_ptr<Planner> make(ProblemDescription &&descr,
                                const std::string &name) const {
    auto it = predicates.find(name);
    if (it == predicates.end()) {
      throw Error{name, " is a non recognized planner type"};
    }
    return it->second(std::forward<ProblemDescription>(descr));
  }

  static PlannerFactory &get() {
    static PlannerFactory res = PlannerFactory{};
    return res;
  }

private:
  PlannerFactory() {
    this->template providePredicate<StandardPlanner>("STANDARD");

    this->template providePredicate<EmbarassinglyParallelPlanner>("EMBARASS");
    this->template providePredicate<ParallelizedQueriesPlanner>("PQUERY");
    this->template providePredicate<SharedTreePlanner>("SHARED");
    this->template providePredicate<LinkedTreesPlanner>("LINKED");
    this->template providePredicate<MultiAgentPlanner>("MULTIAG");
  }

  template <typename PlannerT> void providePredicate(const std::string &name) {
    predicates.emplace(name, [](ProblemDescription &&descr) {
      return std::make_unique<PlannerT>(
          std::forward<ProblemDescription>(descr));
    });
  }

  std::unordered_map<std::string, std::function<std::unique_ptr<Planner>(
                                      ProblemDescription &&)>>
      predicates;
};
} // namespace

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

std::unique_ptr<Planner> makePlanner(ProblemDescription &&desc,
                                     const std::string &type) {
  return PlannerFactory::get().make(std::forward<ProblemDescription>(desc),
                                    type);
}

void setUpPlanner(Planner &planner, const PlannerParameters &params) {
  if (params.threads != 0) {
    auto *maybe_mt = dynamic_cast<MultiThreadedPlanner *>(&planner);
    if (maybe_mt) {
      maybe_mt->setThreads(params.threads);
    }
  }
  if (params.synchronization != 0) {
    auto *maybe_sy = dynamic_cast<SynchronizationAware *>(&planner);
    if (maybe_sy) {
      maybe_sy->synchronization().set(params.synchronization);
    }
  }
}

} // namespace mt_rrt
