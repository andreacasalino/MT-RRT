#include <gtest/gtest.h>

#include <Primitives.h>

using namespace mt_rrt;
using namespace mt_rrt::geom;

#include <LogResult.h>
#include <Logger.h>

static std::unordered_map<std::string, std::size_t> logs;

namespace {
void log_case(const std::string &tag, const Box &box, const Segment &segment) {
  LogResult result;
  result.addObstacle(box);
  result.addObstacle(segment);
  auto it = logs.find(tag);
  if (it == logs.end()) {
    it = logs.emplace(tag, 0).first;
  }
  ++it->second;
  Logger::get().add(tag, "Result-" + std::to_string(it->second), result.get());
}

const Box box = Box{{-1.f, -1.f}, {1.f, 1.f}};
} // namespace

using BoxTestWithCollisionFixture = ::testing::TestWithParam<Segment>;

TEST_P(BoxTestWithCollisionFixture, check_collision) {
  const auto &segment = GetParam();

  log_case("collision", box, segment);

  EXPECT_TRUE(box.collides(segment));
}

INSTANTIATE_TEST_CASE_P(
    BoxTestWithCollisionTests, BoxTestWithCollisionFixture,
    ::testing::Values(Segment{{0, 0}, {2.f, 0}}, Segment{{0, 0}, {2.f, 2.f}},
                      Segment{{0, 0}, {-2.f, -2.f}}, Segment{{0, 0}, {0, -2.f}},
                      Segment{{-2.f, -2.f}, {2.f, 2.f}},
                      Segment{{-2.f, 0}, {2.f, 0}},
                      Segment{{-1.f - 0.05f * 0.1f, -1.f - 0.05f * 0.1f},
                              {-0.9696f, -0.9696f}}));

using BoxTestNoCollisionFixture = ::testing::TestWithParam<Segment>;

TEST_P(BoxTestNoCollisionFixture, check_collision) {
  const auto &segment = GetParam();

  log_case("no-collision", box, segment);

  EXPECT_FALSE(box.collides(segment));
}

INSTANTIATE_TEST_CASE_P(BoxTestNoCollisionTests, BoxTestNoCollisionFixture,
                        ::testing::Values(Segment{{1.5f, 0}, {2.f, 0}},
                                          Segment{{-2.f, 0}, {-1.5f, 0}},
                                          Segment{{1.5f, 1.5f}, {2.f, 2.f}},
                                          Segment{{1.5f, 1.5f}, {2.f, 2.f}},
                                          Segment{{2.f + 0.1f, 0 + 0.1f},
                                                  {0 + 0.1f, 2.f + 0.1f}},
                                          Segment{{2.f + 0.1f, 0 - 0.1f},
                                                  {0 + 0.1f, -2.f - 0.1f}}));
