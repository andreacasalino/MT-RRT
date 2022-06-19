#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "ParallelFor.h"

#include <algorithm>

namespace {
struct Result {
  int val = 0;
};

bool contains(const std::vector<Result> &values, const int to_search) {
  return std::find_if(values.begin(), values.end(),
                      [&to_search](const Result &element) {
                        return element.val == to_search;
                      }) != values.end();
}
} // namespace

TEST_CASE("Parallel for correctness test",
          mt_rrt::merge(TEST_TAG, "[parallel_for]")) {
  using namespace mt_rrt;

  auto threads = GENERATE(2, 3, 4);

  ParallelFor parallel_for(threads);

  std::vector<int> values = {1, 2, -2, 5, 6, -3, 0, 1, 3,
                             1, 2, -2, 5, 6, -3, 0, 1, 3};

  auto values_sorted = values;
  std::sort(values_sorted.begin(), values_sorted.end());

  const std::size_t cycles = 3;

  SECTION("compute min") {
    for (std::size_t c = 0; c < cycles; ++c) {
      const auto mins =
          parallel_for.process<Result, std::vector<int>::const_iterator>(
              values.begin(), values.end(),
              [](Result &res, const int &val, const std::size_t) {
                res.val = std::min(res.val, val);
              });
      CHECK(contains(mins, values_sorted.front()));
    }
  }

  SECTION("compute max") {
    for (std::size_t c = 0; c < cycles; ++c) {
      const auto maxs =
          parallel_for.process<Result, std::vector<int>::const_iterator>(
              values.begin(), values.end(),
              [](Result &res, const int &val, const std::size_t) {
                res.val = std::max(res.val, val);
              });
      CHECK(contains(maxs, values_sorted.back()));
    }
  }
}

TEST_CASE("Parallel for test profile computation times",
          mt_rrt::merge(TEST_TAG, "[parallel_for]")) {
  using namespace mt_rrt;

  Jobs jobs;
  for (std::size_t k = 0; k < 10; ++k) {
    jobs.emplace_back([](const std::size_t) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    });
  }

  auto measure_time = [](const std::function<void()> &action) {
    auto tic = std::chrono::high_resolution_clock::now();
    action();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::high_resolution_clock::now() - tic);
  };

  ParallelFor parallel_for{2};
  const auto time = measure_time([&]() { parallel_for.process(jobs); });
  CHECK(time.count() < 300);
}