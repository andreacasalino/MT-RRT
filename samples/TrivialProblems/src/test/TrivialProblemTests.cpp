#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <MT-RRT-carpet/Strings.h>

#include <TrivialProblem.h>

#ifdef TEST_LOGGING
#include <IO.h>
#endif

namespace {
struct Segment {
  mt_rrt::State start;
  mt_rrt::State end;
};

#ifdef TEST_LOGGING
nlohmann::json log_case(const mt_rrt::samples::Box &box,
                        const Segment &segment) {
  using namespace mt_rrt::utils;
  using namespace mt_rrt::samples;

  nlohmann::json result;

  if (box.trsf) {
    throw mt_rrt::Error{"Can't log box different than AABB"};
  }
  result["box"]["start"] = box.min_corner;
  result["box"]["end"] = box.max_corner;

  result["segment"]["start"] = segment.start;
  result["segment"]["end"] = segment.end;

  result["collides"] = collides(segment.start, segment.end, box);

  return result;
}

static const std::string PYTHON_SCRIPT =
    mt_rrt::merge(TEST_FOLDER, "TrivialProblemTests.py");
#endif
} // namespace

TEST_CASE("check collision check segment-box",
          mt_rrt::merge(TEST_TAG, "[point_problem]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;
  using namespace mt_rrt::samples;

  Box box = Box{{-1.f, -1.f}, {1.f, 1.f}};

  SECTION("collision expected") {
    auto segment = GENERATE(
        Segment{{0, 0}, {2.f, 0}}, Segment{{0, 0}, {2.f, 2.f}},
        Segment{{0, 0}, {-2.f, -2.f}}, Segment{{0, 0}, {0, -2.f}},
        Segment{{-2.f, -2.f}, {2.f, 2.f}}, Segment{{-2.f, 0}, {2.f, 0}},
        Segment{{-1.f - TrivialProblemConnector::STEER_DEGREE * 0.1f,
                 -1.f - TrivialProblemConnector::STEER_DEGREE * 0.1f},
                {-0.9696f, -0.9696f}});

#ifdef TEST_LOGGING
    Logger::log(Logger::Log{ "box_segment_collisions", log_case(box, segment), PYTHON_SCRIPT });
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
    Logger::log(Logger::Log{ "box_segment_collisions", log_case(box, segment), PYTHON_SCRIPT });
#endif
    CHECK_FALSE(collides(segment.start, segment.end, box));
    CHECK_FALSE(collides(segment.end, segment.start, box));
  }
}

namespace {
mt_rrt::State make_state(const float val) { return mt_rrt::State{val, val}; }
} // namespace

TEST_CASE("steer point in plane", mt_rrt::merge(TEST_TAG, "[point_problem]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::utils;
  using namespace mt_rrt::samples;

  TrivialProblemConnector connector({Box{{-1.f, -1.f}, {1.f, 1.f}}});

  SECTION("blocked: no steer at all") {
    Node start(State{-1.f - TrivialProblemConnector::STEER_DEGREE * 0.1f,
                     -1.f - TrivialProblemConnector::STEER_DEGREE * 0.1f});
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
