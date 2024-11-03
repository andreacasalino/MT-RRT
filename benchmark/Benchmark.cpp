#include <TestScenarios.h>

#include <benchmark/benchmark.h>

#include <MT-RRT/EmbarassinglyParallel.h>
#include <MT-RRT/LinkedTreesPlanner.h>
#include <MT-RRT/MultiAgentPlanner.h>
#include <MT-RRT/ParallelizedQueriesPlanner.h>
#include <MT-RRT/SharedTreePlanner.h>
#include <MT-RRT/StandardPlanner.h>

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

template <typename PlannerT> struct Benchmark {};

template <typename PlannerT> struct BenchmarkBase {
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
      ParsPack{{0, 1, 2}, {0, 1}, {100, 200, 500, 1000, 2000, 5000}};

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
struct Benchmark<StandardPlanner> : public benchmark::Fixture,
                                    public BenchmarkBase<StandardPlanner> {
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
    : public benchmark::Fixture,
      public BenchmarkMultiThreaded<EmbarassinglyParallelPlanner> {};
BENCHMARK_TEMPLATE_DEFINE_F(Benchmark, EmbarassinglyParallelTest,
                            EmbarassinglyParallelPlanner)
(benchmark::State &st) { this->run(st); }
BENCHMARK_REGISTER_F(Benchmark, EmbarassinglyParallelTest)
    ->ArgsProduct(Args::multiThreaded);

template <>
struct Benchmark<ParallelizedQueriesPlanner>
    : public benchmark::Fixture,
      public BenchmarkMultiThreaded<ParallelizedQueriesPlanner> {};
BENCHMARK_TEMPLATE_DEFINE_F(Benchmark, ParallelizedQueriesTest,
                            ParallelizedQueriesPlanner)
(benchmark::State &st) { this->run(st); }
BENCHMARK_REGISTER_F(Benchmark, ParallelizedQueriesTest)
    ->ArgsProduct(Args::multiThreaded);

template <>
struct Benchmark<SharedTreePlanner>
    : public benchmark::Fixture,
      public BenchmarkMultiThreaded<SharedTreePlanner> {};
BENCHMARK_TEMPLATE_DEFINE_F(Benchmark, SharedTreeTest, SharedTreePlanner)
(benchmark::State &st) { this->run(st); }
BENCHMARK_REGISTER_F(Benchmark, SharedTreeTest)
    ->ArgsProduct(Args::multiThreaded);

template <>
struct Benchmark<LinkedTreesPlanner>
    : public benchmark::Fixture,
      public BenchmarkMultiThreaded<LinkedTreesPlanner> {};
BENCHMARK_TEMPLATE_DEFINE_F(Benchmark, LinkedTreesTest, LinkedTreesPlanner)
(benchmark::State &st) { this->run(st); }
BENCHMARK_REGISTER_F(Benchmark, LinkedTreesTest)
    ->ArgsProduct(Args::multiThreaded);

template <>
struct Benchmark<MultiAgentPlanner>
    : public benchmark::Fixture,
      public BenchmarkMultiThreaded<MultiAgentPlanner> {
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
