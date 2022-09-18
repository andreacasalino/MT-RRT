#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <MT-RRT-carpet/Error.h>
#include <MT-RRT-core/Sampler.h>

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

TEST_CASE("hyperbox sampler", mt_rrt::merge(TEST_TAG, "[hyper_box]")) {
  using namespace mt_rrt;

  struct Corners {
    State corner_min;
    State corner_max;
  };

  SECTION("negative cases") {
    auto corners = GENERATE(Corners{{}, {}}, Corners{{}, {0}}, Corners{{0}, {}},
                            Corners{{0, 0}, {1, 1, 1}}, Corners{{0}, {-1}},
                            Corners{{0, 1, 2}, {-1, -2, -3}},
                            Corners{{0, 1, 2}, {1, -2, 3}});

    CHECK_THROWS_AS(
        std::make_unique<HyperBox>(corners.corner_min, corners.corner_max),
        Error);
  }

  SECTION("positive cases") {
    auto corners = GENERATE(Corners{{0}, {1}}, Corners{{0, 0}, {1, 1}},
                            Corners{{0, 1, -2, -1}, {1, 2, -1, 1}});

    HyperBox sampler(corners.corner_min, corners.corner_max);

    CHECK(almost_equal(corners.corner_min, sampler.minCorner()));
    CHECK(almost_equal(corners.corner_max, sampler.maxCorner()));
  }
}
