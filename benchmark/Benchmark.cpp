#include <gtest/gtest.h>

#include <TestScenarios.h>

#include <MT-RRT/ScopedTimeDuration.h>

#include <MT-RRT/EmbarassinglyParallel.h>
#include <MT-RRT/LinkedTreesPlanner.h>
#include <MT-RRT/MultiAgentPlanner.h>
#include <MT-RRT/ParallelizedQueriesPlanner.h>
#include <MT-RRT/SharedTreePlanner.h>
#include <MT-RRT/StandardPlanner.h>

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

struct Logger {
public:
  ~Logger() {
    // TODO use meaningful location
    std::ofstream{"/tmp/benchmark.json"} << data.dump(1);
  }

  static Logger &get() {
    static Logger res;
    return res;
  }

  struct Record {
    Record(std::string label)
        : source_{Logger::get()}, label_{std::move(label)} {}

    ~Record() {
      auto &recipient = source_.data[label_];
      if (!recipient.is_array()) {
        recipient = nlohmann::json::array();
      }
      recipient.emplace_back(std::int64_t{duration_.count()});
    }

  private:
    Logger &source_;
    std::chrono::nanoseconds duration_;
    ScopedTimeDuration<std::chrono::nanoseconds> durationGuard_{duration_};
    std::string label_;
  };

private:
  Logger() = default;

  nlohmann::json data;
};

template <typename PlannerT> struct BenchmarkContext {};

template <typename PlannerT> struct Benchmark : ::testing::Test {
  std::string getName(const std::vector<std::int64_t> &parameters) const {
    std::string name{
        ::testing::UnitTest::GetInstance()->current_test_info()->name()};
    for (auto val : parameters) {
      name += '-';
      name += std::to_string(val);
    }
    return name;
  }

  void solve() {
    BenchmarkContext<PlannerT>::args.forEach(
        [this](const std::vector<std::int64_t> &parameters, int iter,
               int iter_tot) {
          auto name = this->getName(parameters);
          std::cout << name << ' ' << iter << '/' << iter_tot;
          ExtendProblem data =
              make_scenario(toKind(parameters[0]), toStrategy(parameters[1]));
          data.suggested_parameters.iterations.set(parameters[2]);
          auto planner =
              BenchmarkContext<PlannerT>::make(data.point_problem, parameters);
          Logger::Record record{std::move(name)};
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

using BenchmarkTypes =
    testing::Types<StandardPlanner, EmbarassinglyParallelPlanner,
                   ParallelizedQueriesPlanner, SharedTreePlanner,
                   LinkedTreesPlanner, MultiAgentPlanner>;
TYPED_TEST_SUITE(Benchmark, BenchmarkTypes);
TYPED_TEST(Benchmark, profile) { this->solve(); }
