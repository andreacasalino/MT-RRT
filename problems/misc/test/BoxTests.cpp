#include <gtest/gtest.h>

#include <JsonConversions.h>
#include <LogResult.h>
#include <Logger.h>
#include <Primitives.h>

using namespace mt_rrt;
using namespace mt_rrt::geom;

static std::unordered_map<std::string, std::size_t> logs;

namespace {
void log_case(const Box &box, const Segment &segment) {
  LogResult result;
  result.addObstacle(box);
  result.addObstacle(segment);
  Logger::get().add_test_result(result.get());
}

const Box box = Box{PointAllocated{-1.f, -1.f}, PointAllocated{1.f, 1.f}};
} // namespace

using BoxTestWithCollisionFixture = ::testing::TestWithParam<Segment>;

TEST_P(BoxTestWithCollisionFixture, check_collision) {
  const auto &segment = GetParam();

  log_case(box, segment);

  EXPECT_TRUE(box.collides(segment));
}

INSTANTIATE_TEST_CASE_P(
    BoxTestWithCollisionTests, BoxTestWithCollisionFixture,
    ::testing::Values(
        Segment{PointAllocated{0, 0}, PointAllocated{2.f, 0}},
        Segment{PointAllocated{0, 0}, PointAllocated{2.f, 2.f}},
        Segment{PointAllocated{0, 0}, PointAllocated{-2.f, -2.f}},
        Segment{PointAllocated{0, 0}, PointAllocated{0, -2.f}},
        Segment{PointAllocated{-2.f, -2.f}, PointAllocated{2.f, 2.f}},
        Segment{PointAllocated{-2.f, 0}, PointAllocated{2.f, 0}},
        Segment{PointAllocated{-1.f - 0.05f * 0.1f, -1.f - 0.05f * 0.1f},
                PointAllocated{-0.9696f, -0.9696f}}));

using BoxTestNoCollisionFixture = ::testing::TestWithParam<Segment>;

TEST_P(BoxTestNoCollisionFixture, check_collision) {
  const auto &segment = GetParam();

  log_case(box, segment);

  EXPECT_FALSE(box.collides(segment));
}

INSTANTIATE_TEST_CASE_P(
    BoxTestNoCollisionTests, BoxTestNoCollisionFixture,
    ::testing::Values(
        Segment{PointAllocated{1.5f, 0}, PointAllocated{2.f, 0}},
        Segment{PointAllocated{-2.f, 0}, PointAllocated{-1.5f, 0}},
        Segment{PointAllocated{1.5f, 1.5f}, PointAllocated{2.f, 2.f}},
        Segment{PointAllocated{1.5f, 1.5f}, PointAllocated{2.f, 2.f}},
        Segment{PointAllocated{2.f + 0.1f, 0 + 0.1f},
                PointAllocated{0 + 0.1f, 2.f + 0.1f}},
        Segment{PointAllocated{2.f + 0.1f, 0 - 0.1f},
                PointAllocated{0 + 0.1f, -2.f - 0.1f}}));
