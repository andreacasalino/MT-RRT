#include <gtest/gtest.h>

#include <MT-RRT/ParallelFor.h>

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

TEST(ParallelForTest, parallel_for_test) {
  using namespace mt_rrt;

  std::size_t threads = 2;

  ParallelFor parallel_for(threads);

  std::vector<int> values = {1, 2, -2, 5, 6, -3, 0, 1, 3,
                             1, 2, -2, 5, 6, -3, 0, 1, 3};

  const std::size_t cycles = 3;

  {
    SCOPED_TRACE("compute min");

    for (std::size_t c = 0; c < cycles; ++c) {
      std::vector<int> results;
      for (std::size_t k = 0; k < threads; ++k) {
        results.push_back(std::numeric_limits<int>::max());
      }
      parallel_for.process(values, [&results](int val, std::size_t th_id) {
        results[th_id] = std::min<int>(results[th_id], val);
      });
      EXPECT_EQ(*std::min_element(results.begin(), results.end()),
                *std::min_element(values.begin(), values.end()));
    }
  }

  {
    SCOPED_TRACE("compute max");

    for (std::size_t c = 0; c < cycles; ++c) {
      std::vector<int> results;
      for (std::size_t k = 0; k < threads; ++k) {
        results.push_back(std::numeric_limits<int>::min());
      }
      parallel_for.process(values, [&results](int val, std::size_t th_id) {
        results[th_id] = std::max<int>(results[th_id], val);
      });
      EXPECT_EQ(*std::max_element(results.begin(), results.end()),
                *std::max_element(values.begin(), values.end()));
    }
  }
}
