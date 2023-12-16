#include <gtest/gtest.h>

#include <MT-RRT/Error.h>
#include <MT-RRT/Limited.h>
#include <MT-RRT/Strings.h>

#include <memory>

namespace {
struct Limits {
  float min;
  float max;
};
} // namespace

using LimitedTestFixture = ::testing::TestWithParam<Limits>;

TEST_P(LimitedTestFixture, limited_on_both_sides) {
  auto [min, max] = GetParam();

  mt_rrt::Limited<float> limited(min, max);

  limited.set(min);
  EXPECT_EQ(limited.get(), min);

  limited.set(max);
  EXPECT_EQ(limited.get(), max);

  const auto middle = 0.5f * (min + max);
  limited.set(middle);
  EXPECT_EQ(limited.get(), middle);
}

INSTANTIATE_TEST_CASE_P(LimitedTests, LimitedTestFixture,
                        ::testing::Values(Limits{-3.f, 6.f}, Limits{3.f, 6.f},
                                          Limits{-6.f, -3.f}));

using LimitedNegativeTestFixture = ::testing::TestWithParam<Limits>;

TEST_P(LimitedNegativeTestFixture, limited_on_both_sides_negative_tests) {
  auto [min, max] = GetParam();

  EXPECT_THROW(std::make_unique<mt_rrt::Limited<float>>(min, max),
               mt_rrt::Error);
}

INSTANTIATE_TEST_CASE_P(LimitedNegativeTests, LimitedNegativeTestFixture,
                        ::testing::Values(Limits{6.f, -3.f}, Limits{6.f, 3.f},
                                          Limits{-3.f, -6.f}));

TEST(LimitedTest, set_outside_bounds) {
  mt_rrt::Limited<float> limited(-3.0, 6.0);

  EXPECT_THROW(limited.set(-4.0), mt_rrt::Error);
  EXPECT_THROW(limited.set(7.0), mt_rrt::Error);
}
