#include <gtest/gtest.h>

#include <TestScenarios.h>

#include <MT-RRT/ScopedTimeDuration.h>
#include <Logger.h>

#if MT_PLANNERS_ENABLED
#include <MT-RRT/EmbarassinglyParallel.h>
#include <MT-RRT/LinkedTreesPlanner.h>
#include <MT-RRT/MultiAgentPlanner.h>
#include <MT-RRT/ParallelizedQueriesPlanner.h>
#include <MT-RRT/SharedTreePlanner.h>
#include <MT-RRT/StandardPlanner.h>
#endif

#include <nlohmann/json.hpp>

#include <fstream>
#include <span>

using namespace mt_rrt;
using namespace mt_rrt::trivial;

Kind toKind(std::int64_t val) {
  Kind res;
  switch (val) {
  case 0:
    res = Kind::Empty;
    break;
  case 1:
    res = Kind::SmallObstacle;
    break;
  default:
    res = Kind::Cluttered;
    break;
  }
  return res;
}

ExpansionStrategy toStrategy(std::int64_t val) {
  ExpansionStrategy res;
  switch (val) {
  case 0:
    res = ExpansionStrategy::Single;
    break;
  default:
    res = ExpansionStrategy::Star;
    break;
  }
  return res;
}

struct Args {
  Args() = default;

  Args &add(std::vector<std::int64_t> pack) {
    args_.emplace_back(std::move(pack));
    return *this;
  }

  Args &iterations(int iters) {
    iterations_ = iters;
    return *this;
  }

  template <typename Pred> void forEach(const Pred &pred) const {
    this->forEach_(std::vector<std::int64_t>{}, args_.begin(), pred);
  }

private:
  template <typename Pred>
  void
  forEach_(std::vector<std::int64_t> cumulated,
           std::vector<std::vector<std::int64_t>>::const_iterator remainingIt,
           const Pred &pred) const {
    if (remainingIt == args_.end()) {
      for (int i = 0; i < iterations_; ++i) {
        pred(cumulated, i + 1, iterations_);
      }
      return;
    }
    for (auto val : *remainingIt) {
      auto cumulated_next = cumulated;
      cumulated_next.push_back(val);
      forEach_(std::move(cumulated_next), remainingIt + 1, pred);
    }
  }

  std::vector<std::vector<std::int64_t>> args_;
  int iterations_{20};
};

struct Records {
public:
  ~Records() {
    const auto logDir = Logger::get().tmpFolderPath();
    std::filesystem::create_directories(logDir);
    for(const auto& [planner, data] : data) {
      std::string resName = planner;
      for(size_t k =0; k<resName.size(); ++k) {
        if(resName[k] == ':') {
          resName[k] = '_';
        }
      }
      resName += ".benchamrk.json";
      std::filesystem::path resPath = logDir / resName;
      std::ofstream{resPath} << data.dump(1);
    }
  }

  static Records &get() {
    static Records res;
    return res;
  }

  struct Record {
    Record(std::string bName, std::string aLabel)
    : benchmarkName{std::move(bName)} 
    , argsLabel{std::move(aLabel)} 
    {}

    ~Record() {
      auto &recipient = Records::get().data[benchmarkName][argsLabel];
      if (!recipient.is_array()) {
        recipient = nlohmann::json::array();
      }
      recipient.emplace_back(std::int64_t{duration_.count()});
    }

  private:
    std::chrono::nanoseconds duration_;
    ScopedTimeDuration<std::chrono::nanoseconds> durationGuard_{duration_};
    std::string benchmarkName;
    std::string argsLabel;
  };

private:
  Records() = default;

  std::unordered_map<std::string, nlohmann::json> data;
};

template <typename PlannerT> struct BenchmarkContext {};

template <typename PlannerT> struct Benchmark : ::testing::Test {
  std::pair<std::string, std::string> getName(const std::vector<std::int64_t> &parameters) const {
    std::pair<std::string, std::string> res;
    res.first = std::string{
        ::testing::UnitTest::GetInstance()->current_test_suite()->type_param()};
    res.second = "args";
    for (auto val : parameters) {
      res.second += '-';
      res.second += std::to_string(val);
    }
    return res;
  }

  void solve() {
    BenchmarkContext<PlannerT>::args.forEach(
        [this](const std::vector<std::int64_t> &parameters, int iter,
               int iter_tot) {
          auto&& [plannerName, argsLabel] = this->getName(parameters);
          std::cout << plannerName << ' ' << argsLabel << ' ' << iter << '/' << iter_tot << std::endl;
          ExtendProblem data =
              make_scenario(toKind(parameters[0]), toStrategy(parameters[1]));
          data.suggested_parameters.iterations.set(parameters[2]);
          auto planner =
              BenchmarkContext<PlannerT>::make(data.point_problem, parameters);
          Records::Record record{std::move(plannerName), std::move(argsLabel)};
          planner->solve(data.start.asView().convert(),
                         data.end.asView().convert(),
                         data.suggested_parameters);
        });
  }
};

template <> struct BenchmarkContext<StandardPlanner> {
  static std::unique_ptr<StandardPlanner>
  make(std::shared_ptr<ProblemDescription> problem,
       const std::vector<std::int64_t> &parameters) {
    return std::make_unique<StandardPlanner>(std::move(*problem));
  }

  static inline const Args args = Args{}.add({0, 1, 2}).add({0, 1}).add(
      {100, 200, 500, 1000, 2000, 5000, 10000});
};

#if MT_PLANNERS_ENABLED
template <typename PlannerT> struct BenchmarkMultiThreadedContext {
  static std::unique_ptr<PlannerT>
  make(std::shared_ptr<ProblemDescription> problem,
       const std::vector<std::int64_t> &parameters) {
    auto planner = std::make_unique<PlannerT>(std::move(*problem));
    planner->setThreads(Threads(parameters.back()));
    return planner;
  }

  static inline const Args args =
      Args{BenchmarkContext<StandardPlanner>::args}.add({2, 3, 4, 6, 8});
};

template <>
struct BenchmarkContext<EmbarassinglyParallelPlanner>
    : BenchmarkMultiThreadedContext<EmbarassinglyParallelPlanner> {};

template <>
struct BenchmarkContext<ParallelizedQueriesPlanner>
    : BenchmarkMultiThreadedContext<ParallelizedQueriesPlanner> {};

template <>
struct BenchmarkContext<SharedTreePlanner>
    : BenchmarkMultiThreadedContext<SharedTreePlanner> {};

template <>
struct BenchmarkContext<LinkedTreesPlanner>
    : BenchmarkMultiThreadedContext<LinkedTreesPlanner> {};

template <>
struct BenchmarkContext<MultiAgentPlanner>
    : BenchmarkMultiThreadedContext<MultiAgentPlanner> {
  static std::unique_ptr<MultiAgentPlanner>
  make(std::shared_ptr<ProblemDescription> problem,
       const std::vector<std::int64_t> &parameters) {
    auto planner = BenchmarkMultiThreadedContext<MultiAgentPlanner>::make(
        problem, parameters);
    planner->synchronization().set(0.1f);
    return planner;
  }
};
#endif

using BenchmarkTypes =
    testing::Types<StandardPlanner
#if MT_PLANNERS_ENABLED
                  ,EmbarassinglyParallelPlanner
                  ,ParallelizedQueriesPlanner
                  ,SharedTreePlanner
                  ,LinkedTreesPlanner
                  ,MultiAgentPlanner
#endif
                   >;
TYPED_TEST_SUITE(Benchmark, BenchmarkTypes);
TYPED_TEST(Benchmark, profile) { this->solve(); }
