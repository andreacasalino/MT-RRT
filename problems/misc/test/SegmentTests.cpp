#include <gtest/gtest.h>

#include <Primitives.h>
#include <memory>

using namespace mt_rrt;
using namespace mt_rrt::geom;

namespace {
bool almost_equal(float a, float b) { return std::abs(a - b) < 0.001f; }

static const Point ORIGIN = Point{0, 0};

struct SegmentTestable {
  std::shared_ptr<Point> start;
  std::shared_ptr<Segment> segment;
};
SegmentTestable make_segment(const Point &start, const Point &end) {
  SegmentTestable result;
  result.start = std::make_shared<Point>(start);
  result.segment = std::make_shared<Segment>(*result.start, end);
  return result;
}

} // namespace

struct SegmentVSPointTestCase {
  SegmentTestable line;
  float s_min_expected;
};

using SegmentVSPointTestFixture =
    ::testing::TestWithParam<SegmentVSPointTestCase>;

TEST_P(SegmentVSPointTestFixture, closest_point_segment_VS_point) {
  auto [line, s_min_expected] = GetParam();

  const auto s_min = line.segment->closest_on_line(ORIGIN);
  EXPECT_TRUE(almost_equal(s_min, s_min_expected));
}

INSTANTIATE_TEST_CASE_P(
    SegmentVSPointTests, SegmentVSPointTestFixture,
    ::testing::Values(
        SegmentVSPointTestCase{make_segment({-1.f, 1.f}, {1.f, 1.f}), 0.5f},
        SegmentVSPointTestCase{make_segment({0.f, 1.f}, {1.f, 1.f}), 0.f},
        SegmentVSPointTestCase{make_segment({-1.f, 1.f}, {0.f, 1.f}), 1.f},
        SegmentVSPointTestCase{make_segment({0.f, 1.f}, {1.f, 0.f}), 0.5f},
        SegmentVSPointTestCase{make_segment({1.f, 0.f}, {0.f, 1.f}), 0.5f}));

struct SegmentVSSegmentTestCase {
  SegmentTestable seg_a;
  SegmentTestable seg_b;
};

using SegmentVSSegmentTestFixture =
    ::testing::TestWithParam<SegmentVSSegmentTestCase>;

TEST_P(SegmentVSSegmentTestFixture,
       closest_point_segment_VS_segment_non_parallel) {
  auto [seg_a, seg_b] = GetParam();

  const auto maybe_pair =
      Segment::closest_on_lines(*seg_a.segment, *seg_b.segment);
  ASSERT_TRUE(maybe_pair);
  auto [s_min, t_min] = maybe_pair.value();

  const auto point_along_a = seg_a.segment->at(s_min);
  const auto point_along_b = seg_b.segment->at(t_min);

  EXPECT_TRUE(almost_equal(point_along_a.data()[0], point_along_b.data()[0]));
  EXPECT_TRUE(almost_equal(point_along_a.data()[1], point_along_b.data()[1]));
}

INSTANTIATE_TEST_CASE_P(
    SegmentVSPointTests, SegmentVSSegmentTestFixture,
    ::testing::Values(
        SegmentVSSegmentTestCase{make_segment({-1.f, -1.f}, {1.f, 1.f}),
                                 make_segment({-1.f, 1.f}, {1.f, -1.f})},
        SegmentVSSegmentTestCase{make_segment({-1.f, -1.f}, {0.f, 0.f}),
                                 make_segment({1.f, 1.f}, {1.f, 0.f})}));

using SegmentVSSegmentParallelTestFixture =
    ::testing::TestWithParam<SegmentVSSegmentTestCase>;

TEST_P(SegmentVSSegmentParallelTestFixture,
       closest_point_segment_VS_segment_parallel) {
  auto [seg_a, seg_b] = GetParam();

  EXPECT_FALSE(Segment::closest_on_lines(*seg_a.segment, *seg_b.segment));
}

INSTANTIATE_TEST_CASE_P(
    SegmentVSSegmentParallelTests, SegmentVSSegmentParallelTestFixture,
    ::testing::Values(
        SegmentVSSegmentTestCase{make_segment({0.f, 0.f}, {1.f, 0.f}),
                                 make_segment({0.f, 1.f}, {1.f, 1.f})},
        SegmentVSSegmentTestCase{make_segment({-1.f, -1.f}, {1.f, 1.f}),
                                 make_segment({-1.f, -1.5f}, {1.f, 0.5f})}));
