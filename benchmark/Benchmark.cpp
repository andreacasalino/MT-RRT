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

/*
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
    Record(Logger &src, benchmark::State &state, int argsNumb)
        : source_{src}, label_{state.name()} {
      for (int k = 0; k < argsNumb; ++k) {
        label_ += '-';
        label_ += std::to_string(state.range(k));
      }
    }

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

  template <typename... ARGS>
  std::unique_ptr<Record> makeRecord(ARGS &&...args) {
    return std::make_unique<Record>(*this, std::forward<ARGS>(args)...);
  }

private:
  Logger() = default;

  nlohmann::json data;
};
*/

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

  template <typename Pred> void forEach(Pred pred) {
    this->forEach_(std::vector<std::int64_t>{}, args_.begin(), pred);
  }

private:
  template <typename Pred>
  void forEach_(std::vector<std::int64_t> cumulated,
                std::vector<std::vector<std::int64_t>>::iterator remainingIt,
                const Pred &pred) {
    if (remainingIt == args_.end()) {
      for (int i = 0; i < iterations_; ++i) {
        pred(cumulated);
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

TEST(Foo, Bla) {
  Args{}
      .add(std::vector<std::int64_t>{0, 1})
      .add(std::vector<std::int64_t>{3, 4, 5})
      .add(std::vector<std::int64_t>{0, 7})
      .forEach([](const std::vector<std::int64_t> &args) {
        for (auto val : args) {
          std::cout << ' ' << val;
        }
        std::cout << std::endl;
      });
}

/*
template <typename PlannerT> struct Benchmark {};

template <typename PlannerT> struct BenchmarkBase : public benchmark::Fixture {
  virtual void setUp(benchmark::State &state) {
    ExtendProblem data =
        make_scenario(toKind(state.range(0)), toStrategy(state.range(1)));

    problem = data.point_problem;
    start = data.start.asView().convert();
    end = data.end.asView().convert();
    pars = std::move(data.suggested_parameters);
    pars.dumpTrees = false;
    pars.best_effort = false;
    pars.iterations.set(state.range(2));
  }

  void run(benchmark::State &state) {
    for (auto _ : state) {
      state.PauseTiming();
      this->setUp(state);
      state.ResumeTiming();
      auto durationGuard = Logger::get().makeRecord(state, this->ArgsCnt());
      planner->solve(start, end, pars);
    }
  }

  std::shared_ptr<ProblemDescription> problem;
  std::vector<float> start;
  std::vector<float> end;
  Parameters pars;
  std::optional<PlannerT> planner;
};

struct Args {
  using ParsPack = std::vector<std::vector<std::int64_t>>;

  static const inline ParsPack standard =
      ParsPack{{0, 1, 2}, {0, 1}, {100, 200, 500, 1000, 2000, 5000, 10000}};

  struct ParsPackMerge {
    ParsPackMerge() = default;
    ParsPackMerge(const ParsPack &begin) : res{begin} {}

    ParsPackMerge &add(std::vector<std::int64_t> &&to_add) {
      res.emplace_back(std::forward<std::vector<std::int64_t>>(to_add));
      return *this;
    }

    ParsPack res;
  };

  static const inline ParsPack multiThreaded =
      ParsPackMerge{standard}.add({2, 3, 4, 6, 8}).res;
};

template <>
struct Benchmark<StandardPlanner> : public BenchmarkBase<StandardPlanner> {
  void setUp(benchmark::State &state) override {
    this->BenchmarkBase<StandardPlanner>::setUp(state);
    this->planner.emplace(std::move(*this->problem));
  }
};

BENCHMARK_TEMPLATE_DEFINE_F(Benchmark, StandardTest, StandardPlanner)
(benchmark::State &st) { this->run(st); }
BENCHMARK_REGISTER_F(Benchmark, StandardTest)->ArgsProduct(Args::standard);

template <typename PlannerT>
struct BenchmarkMultiThreaded : public BenchmarkBase<PlannerT> {
  void setUp(benchmark::State &state) override {
    this->BenchmarkBase<PlannerT>::setUp(state);
    this->planner.emplace(std::move(*this->problem));
    this->planner->setThreads(Threads(state.range(3)));
  }
};

template <>
struct Benchmark<EmbarassinglyParallelPlanner>
    : public BenchmarkMultiThreaded<EmbarassinglyParallelPlanner> {};
BENCHMARK_TEMPLATE_DEFINE_F(Benchmark, EmbarassinglyParallelTest,
                            EmbarassinglyParallelPlanner)
(benchmark::State &st) { this->run(st); }
BENCHMARK_REGISTER_F(Benchmark, EmbarassinglyParallelTest)
    ->ArgsProduct(Args::multiThreaded);

template <>
struct Benchmark<ParallelizedQueriesPlanner>
    : public BenchmarkMultiThreaded<ParallelizedQueriesPlanner> {};
BENCHMARK_TEMPLATE_DEFINE_F(Benchmark, ParallelizedQueriesTest,
                            ParallelizedQueriesPlanner)
(benchmark::State &st) { this->run(st); }
BENCHMARK_REGISTER_F(Benchmark, ParallelizedQueriesTest)
    ->ArgsProduct(Args::multiThreaded);

template <>
struct Benchmark<SharedTreePlanner>
    : public BenchmarkMultiThreaded<SharedTreePlanner> {};
BENCHMARK_TEMPLATE_DEFINE_F(Benchmark, SharedTreeTest, SharedTreePlanner)
(benchmark::State &st) { this->run(st); }
BENCHMARK_REGISTER_F(Benchmark, SharedTreeTest)
    ->ArgsProduct(Args::multiThreaded);

template <>
struct Benchmark<LinkedTreesPlanner>
    : public BenchmarkMultiThreaded<LinkedTreesPlanner> {};
BENCHMARK_TEMPLATE_DEFINE_F(Benchmark, LinkedTreesTest, LinkedTreesPlanner)
(benchmark::State &st) { this->run(st); }
BENCHMARK_REGISTER_F(Benchmark, LinkedTreesTest)
    ->ArgsProduct(Args::multiThreaded);

template <>
struct Benchmark<MultiAgentPlanner>
    : public BenchmarkMultiThreaded<MultiAgentPlanner> {
  void setUp(benchmark::State &state) override {
    this->BenchmarkMultiThreaded<MultiAgentPlanner>::setUp(state);
    this->planner->synchronization().set(0.1f);
  }
};
BENCHMARK_TEMPLATE_DEFINE_F(Benchmark, MultiAgentTest, MultiAgentPlanner)
(benchmark::State &st) { this->run(st); }
BENCHMARK_REGISTER_F(Benchmark, MultiAgentTest)
    ->ArgsProduct(Args::multiThreaded);

// Run the benchmark
BENCHMARK_MAIN();

*/
