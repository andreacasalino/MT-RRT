#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <MT-RRT-carpet/Strings.h>

#include <PlanarRobotsProblem.h>

#ifdef TEST_LOGGING
#include <Logger.h>
#endif

#include <iostream>
#include <limits.h>

namespace {
bool almost_equal(float a, float b) { return std::abs(a - b) < 0.001f; }

mt_rrt::samples::Segment make_segment(const mt_rrt::samples::Point &a,
                                      const mt_rrt::samples::Point &b) {
  mt_rrt::samples::Segment segment;
  segment.front() = std::make_shared<mt_rrt::samples::Point>(a);
  segment.back() = std::make_shared<mt_rrt::samples::Point>(b);
  return segment;
}

static const mt_rrt::samples::Point ORIGIN = mt_rrt::samples::Point{0, 0};

std::ostream &operator<<(std::ostream &s, const mt_rrt::samples::Point &p) {
  s << '<' << p.at(0) << " , " << p.at(1) << '>';
  return s;
}

std::ostream &operator<<(std::ostream &s, const mt_rrt::samples::Robot &r) {
  s << *r.getBody().front().segment.at(0) << '\n';
  for (const auto &link : r.getBody()) {
    s << *link.segment.at(1) << '\n';
  }
  return s;
}
} // namespace

TEST_CASE("check closest point segment VS point",
          mt_rrt::merge(TEST_TAG, "[segment-point-closest]")) {
  using namespace mt_rrt::samples;

  struct TestCase {
    Segment line;
    float s_min_expected;
  };

  auto test_case =
      GENERATE(TestCase{make_segment({-1.f, 1.f}, {1.f, 1.f}), 0.5f},
               TestCase{make_segment({0.f, 1.f}, {1.f, 1.f}), 0.f},
               TestCase{make_segment({-1.f, 1.f}, {0.f, 1.f}), 1.f},
               TestCase{make_segment({0.f, 1.f}, {1.f, 0.f}), 0.5f},
               TestCase{make_segment({1.f, 0.f}, {0.f, 1.f}), 0.5f});

  const auto s_min = closest_point(test_case.line, ORIGIN);
  CHECK(almost_equal(s_min, test_case.s_min_expected));
}

namespace {
mt_rrt::samples::Point track_on_segment(const mt_rrt::samples::Segment &seg,
                                        const float s) {
  mt_rrt::samples::Point result = *seg.front();
  result.at(0) += s * (seg.back()->at(0) - seg.front()->at(0));
  result.at(1) += s * (seg.back()->at(1) - seg.front()->at(1));
  return result;
}
} // namespace

TEST_CASE("check closest pair segment VS segment",
          mt_rrt::merge(TEST_TAG, "[segment-segment-closest]")) {
  using namespace mt_rrt::samples;

  struct TestCase {
    Segment seg_a;
    Segment seg_b;
  };

  SECTION("non parallel segments") {
    auto test_case = GENERATE(TestCase{make_segment({-1.f, -1.f}, {1.f, 1.f}),
                                       make_segment({-1.f, 1.f}, {1.f, -1.f})},
                              TestCase{make_segment({-1.f, -1.f}, {0.f, 0.f}),
                                       make_segment({1.f, 1.f}, {1.f, 0.f})});

    const auto maybe_pair = closest_pair(test_case.seg_a, test_case.seg_b);
    REQUIRE(maybe_pair);
    const auto s_min = maybe_pair->front();
    const auto t_min = maybe_pair->back();

    const auto point_along_a = track_on_segment(test_case.seg_a, s_min);
    const auto point_along_b = track_on_segment(test_case.seg_b, t_min);

    CHECK(almost_equal(point_along_a.at(0), point_along_b.at(0)));
    CHECK(almost_equal(point_along_a.at(1), point_along_b.at(1)));
  }

  SECTION("parallel segments") {
    auto test_case =
        GENERATE(TestCase{make_segment({0.f, 0.f}, {1.f, 0.f}),
                          make_segment({0.f, 1.f}, {1.f, 1.f})},
                 TestCase{make_segment({-1.f, -1.f}, {1.f, 1.f}),
                          make_segment({-1.f, -1.5f}, {1.f, 0.5f})});

    CHECK_FALSE(closest_pair(test_case.seg_a, test_case.seg_b));
  }
}

namespace {
mt_rrt::samples::Robot make_robot(const std::size_t dof, float ray = 0.1f,
                                  const mt_rrt::samples::Point &base = {0, 0}) {
  std::vector<mt_rrt::samples::Robot::Link> links;
  for (std::size_t k = 0; k < dof; ++k) {
    auto &link = links.emplace_back();
    link.length.set(1.f);
    link.ray.set(ray);
  }
  return mt_rrt::samples::Robot{links, base};
}

mt_rrt::State make_end_pose(const mt_rrt::State &start) {
  auto result = start;
  for (auto &q : result) {
    q += 1.f;
  }
  return result;
}
} // namespace

TEST_CASE("check Robot::setPose", mt_rrt::merge(TEST_TAG, "[planer-robot]")) {
  using namespace mt_rrt::samples;

  auto robot = make_robot(3, 0);

  SECTION("<0,0,0> pose") {
    robot.setPose({0, 0, 0});

    CHECK(almost_equal(robot.getBody().at(0).segment.at(1)->at(0), 1.f));
    CHECK(almost_equal(robot.getBody().at(0).segment.at(1)->at(1), 0));

    CHECK(almost_equal(robot.getBody().at(1).segment.at(1)->at(0), 2.f));
    CHECK(almost_equal(robot.getBody().at(1).segment.at(1)->at(1), 0));

    CHECK(almost_equal(robot.getBody().at(2).segment.at(1)->at(0), 3.f));
    CHECK(almost_equal(robot.getBody().at(2).segment.at(1)->at(1), 0));
  }

  SECTION("<0,90,0> pose") {
    robot.setPose({0, to_rad(90.f), 0});

    CHECK(almost_equal(robot.getBody().at(0).segment.at(1)->at(0), 1.f));
    CHECK(almost_equal(robot.getBody().at(0).segment.at(1)->at(1), 0));

    CHECK(almost_equal(robot.getBody().at(1).segment.at(1)->at(0), 1.f));
    CHECK(almost_equal(robot.getBody().at(1).segment.at(1)->at(1), 1.f));

    CHECK(almost_equal(robot.getBody().at(2).segment.at(1)->at(0), 1.f));
    CHECK(almost_equal(robot.getBody().at(2).segment.at(1)->at(1), 2.f));
  }
}

TEST_CASE("check Robot::delta", mt_rrt::merge(TEST_TAG, "[planer-robot]")) {
  using namespace mt_rrt::samples;

  auto robot = make_robot(3, 0);

  SECTION("<0,0,0> pose") {
    mt_rrt::State start{0, 0, 0};
    const auto delta = robot.delta(start, make_end_pose(start));
    CHECK(almost_equal(delta, 6.f));
  }

  SECTION("<90,90,90> pose") {
    mt_rrt::State start{to_rad(90.f), to_rad(90.f), to_rad(90.f)};
    const auto delta = robot.delta(start, make_end_pose(start));
    const auto delta_expected = sqrtf(2.f) + sqrtf(2.f) + 1.f;
    CHECK(almost_equal(delta, delta_expected));
  }

  SECTION("<0,90,0> pose") {
    mt_rrt::State start{0, to_rad(90.f), 0};
    const auto delta = robot.delta(start, make_end_pose(start));
    const auto delta_expected = sqrtf(5.f) + 2.f + 1.f;
    CHECK(almost_equal(delta, delta_expected));
  }
}

TEST_CASE("check Robot::distance", mt_rrt::merge(TEST_TAG, "[planer-robot]")) {
  using namespace mt_rrt::samples;

  const float ray = 0.1f;

  SECTION("distance robot obsacles") {
    Spheres spheres;
    {
      auto &sphere = spheres.emplace_back();
      sphere.ray.set(ray);
      sphere.center = {2.f, 1.f};
    }
    {
      auto &sphere = spheres.emplace_back();
      sphere.ray.set(ray);
      sphere.center = {2.f, -0.5f};
    }

    auto robot = make_robot(2, ray);
    robot.setPose({0, 0});
    const auto distance = robot.distance(spheres);
    CHECK(almost_equal(distance, (0.5 - 2 * ray)));
  }

  SECTION("distance robot robot") {
    auto robot_a = make_robot(3, ray);
    robot_a.setPose({0, to_rad(90), to_rad(90)});

    auto robot_b = make_robot(2, ray, {3.f, 0.5f});
    robot_b.setPose({PI, -to_rad(90)});

    const auto distance = robot_a.distance(robot_b);
    CHECK(almost_equal(distance, (1.f - 2 * ray)));
  }
}

TEST_CASE("check bubble advance", mt_rrt::merge(TEST_TAG, "[planer-robot]")) {
  using namespace mt_rrt;
  using namespace mt_rrt::samples;

  const float sphere_ray = 1.f;
  const float robot_capsules_ray = 0.1f;

  PosesConnector connector;
  auto &scene = *connector.scene;
  scene.robots.emplace_back(make_robot(3, robot_capsules_ray));

  SECTION("expect advanced with obstacle in the middle") {
    {
      auto &sphere = scene.obstacles.emplace_back();
      sphere.ray.set(sphere_ray);
      sphere.center = Point{0, 3.f};
    };
    const auto start = State{to_rad(45), to_rad(-5), to_rad(-10)};
    const auto target = State{to_rad(135), to_rad(5), to_rad(10)};

    auto traj = connector.getTrajectory(start, target);
    CHECK(traj->getState() == start);

    auto traj_status = traj->advance();
    CHECK(traj_status == AdvanceInfo::advanced);

    const auto reached = traj->getState();
    CHECK(reached[0] < to_rad(90));

    // check all joints are in between start and target
    for (std::size_t k = 0; k < 3; ++k) {
      CHECK(reached[k] > start[k]);
      CHECK(reached[k] < target[k]);
    }

    SECTION("try an additional steer") {
      auto traj_status = traj->advance();
      CHECK(traj_status == AdvanceInfo::advanced);

      const auto reached2 = traj->getState();
      CHECK(reached2[0] < to_rad(90));

      // check all joints are in between start and target
      for (std::size_t k = 0; k < 3; ++k) {
        CHECK(reached2[k] > reached[k]);
        CHECK(reached2[k] < target[k]);
      }
    }
  }

  SECTION("expect blocked") {
    {
      auto &sphere = scene.obstacles.emplace_back();
      sphere.ray.set(sphere_ray);
      sphere.center = Point{robot_capsules_ray + sphere_ray + 0.00001f, 3.f};
    };
    const auto start = State{to_rad(90), 0, 0};
    const auto target = State{to_rad(135), to_rad(5), to_rad(10)};

    auto traj = connector.getTrajectory(start, target);

    auto traj_status = traj->advance();
    CHECK(traj_status == AdvanceInfo::blocked);

    CHECK(traj->getState() == start);
  }

  SECTION("expect target reached with no obstacles") {
    const auto start = State{to_rad(45), to_rad(-5), to_rad(-10)};
    const auto target = State{to_rad(135), to_rad(5), to_rad(10)};

    auto traj = connector.getTrajectory(start, target);

    auto traj_status = traj->advance();
    CHECK(traj_status == AdvanceInfo::targetReached);

    CHECK(traj->getState() == target);
  }

  SECTION("expect target reached in 1 steer") {
    {
      auto &sphere = scene.obstacles.emplace_back();
      sphere.ray.set(sphere_ray);
      sphere.center = Point{0, 10.f};
    };
    const auto start = State{to_rad(45), to_rad(-5), to_rad(-10)};
    const auto target = State{to_rad(135), to_rad(5), to_rad(10)};

    auto traj = connector.getTrajectory(start, target);

    auto traj_status = traj->advance();
    CHECK(traj_status == AdvanceInfo::targetReached);

    CHECK(traj->getState() == target);
  }

  SECTION("expect target reached after many steer") {
    {
      auto &sphere = scene.obstacles.emplace_back();
      sphere.ray.set(sphere_ray);
      sphere.center = Point{0, 3.f + robot_capsules_ray + sphere_ray + 0.1f};
    };
    const auto start = State{to_rad(45), to_rad(-5), to_rad(-10)};
    const auto target = State{to_rad(135), to_rad(5), to_rad(10)};

    auto traj = connector.getTrajectory(start, target);

    std::size_t iter = 0;
    std::vector<State> intermediate_states;
    while (iter < 100) {
      auto traj_status = traj->advance();
      REQUIRE(traj_status != AdvanceInfo::blocked);
      if (traj_status == AdvanceInfo::targetReached) {
        break;
      }
      intermediate_states.emplace_back(traj->getState());
    }

    // check that distance to target monotonically decreased over iterations
    State previous_distance_to_target;
    for (std::size_t k = 0; k < target.size(); ++k) {
      previous_distance_to_target.push_back(std::numeric_limits<float>::max());
    }
    for (const auto &intermediate_state : intermediate_states) {
      State distance_to_target;
      for (std::size_t k = 0; k < target.size(); ++k) {
        distance_to_target.push_back(
            std::abs(intermediate_state[k] - target[k]));
        CHECK(distance_to_target[k] <= previous_distance_to_target[k]);
      }
      previous_distance_to_target = distance_to_target;
    }

    CHECK(traj->getState() == target);
  }
}
