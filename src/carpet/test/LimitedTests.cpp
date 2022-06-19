#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <MT-RRT-carpet/Error.h>
#include <MT-RRT-carpet/Limited.h>

#include <memory>

namespace {
struct Limits {
  float min;
  float max;
};
} // namespace

TEST_CASE("limited on both sides", TEST_TAG) {
  auto limits =
      GENERATE(Limits{-3.f, 6.f}, Limits{3.f, 6.f}, Limits{-6.f, -3.f});

  mt_rrt::Limited<float> limited(limits.min, limits.max);

  limited.set(limits.min);
  CHECK(limited.get() == limits.min);

  limited.set(limits.max);
  CHECK(limited.get() == limits.max);

  const auto middle = 0.5f * (limits.min + limits.max);
  limited.set(middle);
  CHECK(limited.get() == middle);
}

TEST_CASE("limited on both sides negative tests", TEST_TAG) {

  SECTION("c'tor") {
    auto limits =
        GENERATE(Limits{6.f, -3.f}, Limits{6.f, 3.f}, Limits{-3.f, -6.f});

    CHECK_THROWS_AS(
        std::make_unique<mt_rrt::Limited<float>>(limits.min, limits.max),
        mt_rrt::Error);
  }

  SECTION("set") {
    mt_rrt::Limited<float> limited(-3.0, 6.0);

    CHECK_THROWS_AS(limited.set(-4.0), mt_rrt::Error);
    CHECK_THROWS_AS(limited.set(7.0), mt_rrt::Error);
  }
}
