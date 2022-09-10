#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <MT-RRT-carpet/Strings.h>

#include <Geometry.h>

#ifdef TEST_LOGGING
#include <Logger.h>
#endif

namespace {
bool almost_equal(float a, float b) { return std::abs(a - b) < 0.001f; }

static const mt_rrt::utils::Point ORIGIN = mt_rrt::utils::Point{0, 0};

struct SegmentTestable {
  std::shared_ptr<mt_rrt::utils::Point> start;
  std::shared_ptr<mt_rrt::utils::Segment> segment;
};
SegmentTestable make_segment(const mt_rrt::utils::Point &start,
                             const mt_rrt::utils::Point &end) {
  SegmentTestable result;
  result.start = std::make_shared<mt_rrt::utils::Point>(start);
  result.segment = std::make_shared<mt_rrt::utils::Segment>(*result.start, end);
  return result;
}
} // namespace

TEST_CASE("check closest point segment VS point",
          mt_rrt::merge(TEST_TAG, "[segment-point-closest]")) {
  using namespace mt_rrt::utils;

  struct TestCase {
    SegmentTestable line;
    float s_min_expected;
  };

  auto test_case =
      GENERATE(TestCase{make_segment({-1.f, 1.f}, {1.f, 1.f}), 0.5f},
               TestCase{make_segment({0.f, 1.f}, {1.f, 1.f}), 0.f},
               TestCase{make_segment({-1.f, 1.f}, {0.f, 1.f}), 1.f},
               TestCase{make_segment({0.f, 1.f}, {1.f, 0.f}), 0.5f},
               TestCase{make_segment({1.f, 0.f}, {0.f, 1.f}), 0.5f});

  const auto s_min = closest_on_line(ORIGIN, *test_case.line.segment);
  CHECK(almost_equal(s_min, test_case.s_min_expected));
}

TEST_CASE("check closest pair segment VS segment",
          mt_rrt::merge(TEST_TAG, "[segment-segment-closest]")) {
  using namespace mt_rrt::utils;

  struct TestCase {
    SegmentTestable seg_a;
    SegmentTestable seg_b;
  };

  SECTION("non parallel segments") {
    auto test_case = GENERATE(TestCase{make_segment({-1.f, -1.f}, {1.f, 1.f}),
                                       make_segment({-1.f, 1.f}, {1.f, -1.f})},
                              TestCase{make_segment({-1.f, -1.f}, {0.f, 0.f}),
                                       make_segment({1.f, 1.f}, {1.f, 0.f})});

    const auto maybe_pair =
        closest_on_lines(*test_case.seg_a.segment, *test_case.seg_b.segment);
    REQUIRE(maybe_pair);
    const auto s_min = maybe_pair->front();
    const auto t_min = maybe_pair->back();

    const auto point_along_a =
        point_on_segment(s_min, *test_case.seg_a.segment);
    const auto point_along_b =
        point_on_segment(t_min, *test_case.seg_b.segment);

    CHECK(almost_equal(point_along_a.at(0), point_along_b.at(0)));
    CHECK(almost_equal(point_along_a.at(1), point_along_b.at(1)));
  }

  SECTION("parallel segments") {
    auto test_case =
        GENERATE(TestCase{make_segment({0.f, 0.f}, {1.f, 0.f}),
                          make_segment({0.f, 1.f}, {1.f, 1.f})},
                 TestCase{make_segment({-1.f, -1.f}, {1.f, 1.f}),
                          make_segment({-1.f, -1.5f}, {1.f, 0.5f})});

    CHECK_FALSE(
        closest_on_lines(*test_case.seg_a.segment, *test_case.seg_b.segment));
  }
}
