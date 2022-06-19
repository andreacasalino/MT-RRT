#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <MT-RRT-carpet/Strings.h>

#include <PointProblem.h>

#ifdef TEST_LOGGING
#include <JsonConvert.h>
#include <Logger.h>
#endif

namespace {
struct Segment {
  mt_rrt::State start;
  mt_rrt::State end;
};

#ifdef TEST_LOGGING
nlohmann::json log_case(const mt_rrt::utils::Box &box, const Segment &segment) {
  using namespace mt_rrt::utils;

  nlohmann::json result;
  result["box"] = box;
  result["segment"]["start"] = segment.start;
  result["segment"]["end"] = segment.end;
  result["collides"] = collides(segment.start, segment.end, box);

  return result;
}

static const std::string PYTHON_SCRIPT =
    mt_rrt::merge(TEST_FOLDER, "PointProblemTests.py");
#endif
} // namespace

TEST_CASE("check collision check segment-box 2D",
          mt_rrt::merge(TEST_TAG, "[point_problem]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;

  Box box = Box{{-1.f, -1.f}, {1.f, 1.f}};

  SECTION("collision expected") {
    auto segment = GENERATE(
        Segment{{0, 0}, {2.f, 0}}, Segment{{0, 0}, {2.f, 2.f}},
        Segment{{0, 0}, {-2.f, -2.f}}, Segment{{0, 0}, {0, -2.f}},
        Segment{{-2.f, -2.f}, {2.f, 2.f}}, Segment{{-2.f, 0}, {2.f, 0}},
        Segment{{-1.f - PointConnector::STEER_DEGREE * 0.1f,
                 -1.f - PointConnector::STEER_DEGREE * 0.1f},
                {-0.9696f, -0.9696f}});

#ifdef TEST_LOGGING
    Logger::log("box_segment_collisions", log_case(box, segment),
                PYTHON_SCRIPT);
#endif
    CHECK(collides(segment.start, segment.end, box));
    CHECK(collides(segment.end, segment.start, box));
  }

  SECTION("no collision expected") {
    auto segment = GENERATE(
        Segment{{1.5f, 0}, {2.f, 0}}, Segment{{-2.f, 0}, {-1.5f, 0}},
        Segment{{1.5f, 1.5f}, {2.f, 2.f}}, Segment{{1.5f, 1.5f}, {2.f, 2.f}},
        Segment{{2.f + 0.1f, 0 + 0.1f}, {0 + 0.1f, 2.f + 0.1f}},
        Segment{{2.f + 0.1f, 0 - 0.1f}, {0 + 0.1f, -2.f - 0.1f}});

#ifdef TEST_LOGGING
    Logger::log("box_segment_collisions", log_case(box, segment),
                PYTHON_SCRIPT);
#endif
    CHECK_FALSE(collides(segment.start, segment.end, box));
    CHECK_FALSE(collides(segment.end, segment.start, box));
  }
}

namespace {
mt_rrt::State make_state(const float val, const std::size_t size) {
  mt_rrt::State result;
  result.reserve(size);
  for (std::size_t k = 0; k < size; ++k) {
    result.push_back(val);
  }
  return result;
}
} // namespace

TEST_CASE("check collision check segment-box nD",
          mt_rrt::merge(TEST_TAG, "[point_problem]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;

  auto size = GENERATE(3, 5, 10);

  Box box = Box{make_state(-1.f, size), make_state(1.f, size)};

  SECTION("collision expected") {
    auto segment = Segment{make_state(0, size), make_state(2.f, size)};

#ifdef TEST_LOGGING
    Logger::log("box_segment_collisions", log_case(box, segment),
                PYTHON_SCRIPT);
#endif
    CHECK(collides(segment.start, segment.end, box));
    CHECK(collides(segment.end, segment.start, box));
  }

  SECTION("no collision expected") {
    auto segment = Segment{make_state(1.5f, size), make_state(2.f, size)};

#ifdef TEST_LOGGING
    Logger::log("box_segment_collisions", log_case(box, segment),
                PYTHON_SCRIPT);
#endif
    CHECK_FALSE(collides(segment.start, segment.end, box));
    CHECK_FALSE(collides(segment.end, segment.start, box));
  }
}

TEST_CASE("steer point in 2D", mt_rrt::merge(TEST_TAG, "[point_problem]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;

  PointConnector connector({Box{{-1.f, -1.f}, {1.f, 1.f}}});

  SECTION("blocked: no steer at all") {
    Node start(State{-1.f - PointConnector::STEER_DEGREE * 0.1f,
                     -1.f - PointConnector::STEER_DEGREE * 0.1f});
    SteerIterations trials{10000};
    auto steered = connector.steer(start, State{2.f, 2.f}, trials);

    REQUIRE_FALSE(steered);
  }

  Node start(State{-2.f, -2.f});

  SECTION("advanced") {
    SteerIterations trials{10000};
    auto steered = connector.steer(start, State{2.f, 2.f}, trials);

    REQUIRE(steered);
    const auto *node = steered->steered_node.get();
    REQUIRE(nullptr != node);
    const auto &node_state = node->getState();
    // should have been blocked to a coordinate similar to (-val, -val)
    CHECK_FALSE(steered->target_is_reached);
    CHECK(node->getFatherInfo().father == &start);
    REQUIRE(node_state.size() == 2);
    CHECK(fabs(node_state[0] - node_state[1]) < 1e-4f);
    CHECK(node_state[0] < -1.f);
  }

  SECTION("target reached") {
    const State target = State{-2.f, 2.f};
    SteerIterations trials{10000};
    auto steered = connector.steer(start, target, trials);

    REQUIRE(steered);
    const auto *node = steered->steered_node.get();
    REQUIRE(nullptr != node);
    const auto &node_state = node->getState();
    // target should have been reached
    CHECK(steered->target_is_reached);
    CHECK(node->getFatherInfo().father == &start);
    REQUIRE(node_state.size() == 2);
    CHECK(node_state == target);
  }
}
