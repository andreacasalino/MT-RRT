#include <gtest/gtest.h>

#include <MT-RRT/Error.h>
#include <MT-RRT/HyperBox.h>

namespace {
bool almost_equal(const std::vector<float> &a, const std::vector<float> &b) {
  if (a.size() != b.size()) {
    return false;
  }
  for (std::size_t k = 0; k < a.size(); ++k) {
    if (fabs(a[k] - b[k]) > 1e-5) {
      return false;
    }
  }
  return true;
}
} // namespace

using namespace mt_rrt;

struct Corners {
  std::vector<float> corner_min;
  std::vector<float> corner_max;
};

using HyperBoxSamplerNegativeFixture = ::testing::TestWithParam<Corners>;

TEST_P(HyperBoxSamplerNegativeFixture, negative_tests) {
  auto [corner_min, corner_max] = GetParam();

  EXPECT_THROW(std::make_unique<HyperBox>(corner_min, corner_max), Error);
}

INSTANTIATE_TEST_CASE_P(
    HyperBoxSamplerNegativeTests, HyperBoxSamplerNegativeFixture,
    ::testing::Values(Corners{{}, {}}, Corners{{}, {0}}, Corners{{0}, {}},
                      Corners{{0, 0}, {1, 1, 1}}, Corners{{0}, {-1}},
                      Corners{{0, 1, 2}, {-1, -2, -3}},
                      Corners{{0, 1, 2}, {1, -2, 3}}));

using HyperBoxSamplerPositiveFixture = ::testing::TestWithParam<Corners>;

TEST_P(HyperBoxSamplerPositiveFixture, Positive_tests) {
  auto [corner_min, corner_max] = GetParam();

  HyperBox sampler(corner_min, corner_max);

  EXPECT_TRUE(almost_equal(corner_min, sampler.minCorner()));
  EXPECT_TRUE(almost_equal(corner_max, sampler.maxCorner()));
}

INSTANTIATE_TEST_CASE_P(
    HyperBoxSamplerPositiveTests, HyperBoxSamplerPositiveFixture,
    ::testing::Values(Corners{{0}, {1}}, Corners{{0, 0}, {1, 1}},
                      Corners{{0, 1, -2, -1}, {1, 2, -1, 1}}));

TEST(HyperBoxSamplingTest, samples_remain_in_box) {
  std::vector<float> min_corner{-1.f, -2.f};
  std::vector<float> max_corner{2.f, 1.5f};

  HyperBox sampler(min_corner, max_corner);

  std::vector<float> buffer;
  for (std::size_t i = 0; i < 1000; ++i) {
    sampler.sampleState(buffer);

    ASSERT_EQ(buffer.size(), 2);
    ASSERT_TRUE(min_corner[0] <= buffer[0]);
    ASSERT_TRUE(buffer[0] <= max_corner[0]);
    ASSERT_TRUE(min_corner[1] <= buffer[1]);
    ASSERT_TRUE(buffer[1] <= max_corner[1]);
  }
}