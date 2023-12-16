#include <gtest/gtest.h>

#include <PlanarRobotsProblem.h>

#include <iostream>
#include <limits.h>

using namespace mt_rrt;
using namespace mt_rrt::geom;
using namespace mt_rrt::robots;

namespace {
bool almost_equal(float a, float b) { return std::abs(a - b) < 0.001f; }

std::ostream &operator<<(std::ostream &s, const geom::Point &p) {
  s << '<' << p.data()[0] << " , " << p.data()[1] << '>';
  return s;
}

std::ostream &operator<<(std::ostream &s, const Robot &robot) {
  const auto &body = robot.getBody();
  s << body.front().segment.getStart() << std::endl;
  for (const auto &link : body) {
    s << link.segment.getEnd() << std::endl;
  }
  return s;
}

struct RobotMaker {
  RobotMaker() = default;

  RobotMaker &dof(std::size_t dof) {
    dof_ = dof;
    return *this;
  }
  RobotMaker &ray(float ray) {
    ray_ = ray;
    return *this;
  }
  RobotMaker &base(const Robot::Base &base) {
    base_ = base;
    return *this;
  }

  Robot make() const {
    std::vector<Joint> joints;
    for (std::size_t k = 0; k < dof_; ++k) {
      auto &added = joints.emplace_back();
      added.ray = ray_;
      added.length = 1.f;
    }
    return Robot{joints, base_};
  }

private:
  std::size_t dof_;
  float ray_ = 0.1f;
  Robot::Base base_;
};

std::vector<float> make_end_pose(const std::vector<float> &start) {
  auto result = start;
  for (auto &q : result) {
    q += 1.f;
  }
  return result;
}
} // namespace

TEST(PlanarRobotsTest, setPose) {
  auto robot = RobotMaker{}.dof(3).make();

  {
    SCOPED_TRACE("<0,0,0> pose");

    robot.setPose(std::vector<float>{0, 0, 0});

    std::cout << "----------------------" << std::endl << robot << std::endl;

    EXPECT_TRUE(
        almost_equal(robot.getBody().at(0).segment.getEnd().data()[0], 1.f));
    EXPECT_TRUE(
        almost_equal(robot.getBody().at(0).segment.getEnd().data()[1], 0));

    EXPECT_TRUE(
        almost_equal(robot.getBody().at(1).segment.getEnd().data()[0], 2.f));
    EXPECT_TRUE(
        almost_equal(robot.getBody().at(1).segment.getEnd().data()[1], 0));

    EXPECT_TRUE(
        almost_equal(robot.getBody().at(2).segment.getEnd().data()[0], 3.f));
    EXPECT_TRUE(
        almost_equal(robot.getBody().at(2).segment.getEnd().data()[1], 0));
  }

  {
    SCOPED_TRACE("<0,90,0> pose");

    robot.setPose(std::vector<float>{0, to_rad(90.f), 0});

    std::cout << "----------------------" << std::endl << robot << std::endl;

    EXPECT_TRUE(
        almost_equal(robot.getBody().at(0).segment.getEnd().data()[0], 1.f));
    EXPECT_TRUE(
        almost_equal(robot.getBody().at(0).segment.getEnd().data()[1], 0));

    EXPECT_TRUE(
        almost_equal(robot.getBody().at(1).segment.getEnd().data()[0], 1.f));
    EXPECT_TRUE(
        almost_equal(robot.getBody().at(1).segment.getEnd().data()[1], 1.f));

    EXPECT_TRUE(
        almost_equal(robot.getBody().at(2).segment.getEnd().data()[0], 1.f));
    EXPECT_TRUE(
        almost_equal(robot.getBody().at(2).segment.getEnd().data()[1], 2.f));
  }
}

TEST(PlanarRobotsTest, delta) {
  auto robot = RobotMaker{}.dof(3).make();

  {
    SCOPED_TRACE("<0,0,0> pose");

    std::vector<float> start{0, 0, 0};
    const auto delta = robot.delta(start, make_end_pose(start));
    EXPECT_TRUE(almost_equal(delta, 6.f + 0.3f));
  }

  {
    SCOPED_TRACE("<90,90,90> pose");

    std::vector<float> start{to_rad(90.f), to_rad(90.f), to_rad(90.f)};
    const auto delta = robot.delta(start, make_end_pose(start));
    const auto delta_expected = sqrtf(2.f) + sqrtf(2.f) + 1.f;
    EXPECT_TRUE(almost_equal(delta, delta_expected + 0.3f));
  }

  {
    SCOPED_TRACE("<0,90,0> pose");

    std::vector<float> start{0, to_rad(90.f), 0};
    const auto delta = robot.delta(start, make_end_pose(start));
    const auto delta_expected = sqrtf(5.f) + 2.f + 1.f;
    EXPECT_TRUE(almost_equal(delta, delta_expected + 0.3f));
  }
}

TEST(PlanarRobotsTest, distance_robot_VS_obstacles) {
  float ray = 0.1f;

  std::vector<geom::Sphere> spheres;
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

  auto robot = RobotMaker{}.dof(2).ray(ray).make();
  robot.setPose(std::vector<float>{0, 0});
  float distance = robot.distance(spheres.begin(), spheres.end());
  EXPECT_TRUE(almost_equal(distance, (0.5 - 2 * ray)));
}

TEST(PlanarRobotsTest, distance_robot_VS_robot) {
  float ray = 0.1f;

  auto robot_a = RobotMaker{}.dof(3).ray(ray).make();
  robot_a.setPose(std::vector<float>{0, to_rad(90), to_rad(90)});

  auto robot_b = RobotMaker{}
                     .dof(2)
                     .ray(ray)
                     .base(Robot::Base{geom::Point{3.f, 0.5f}})
                     .make();
  robot_b.setPose(std::vector<float>{PI, -to_rad(90)});

  float distance = robot_a.distance(robot_b);
  EXPECT_TRUE(almost_equal(distance, (1.f - 2 * ray)));
}

class PlanarRobotsTestBubbleAdvance : public ::testing::Test {
protected:
  void SetUp() {
    scene.robots.emplace_back(
        RobotMaker{}.dof(3).ray(robot_capsules_ray).make());
  }

  const float sphere_ray = 1.f;
  const float robot_capsules_ray = 0.1f;

  PosesConnector connector;
  Scene &scene = *connector.scene;
};

TEST_F(PlanarRobotsTestBubbleAdvance, obstacle_in_the_middle) {
  {
    auto &sphere = scene.obstacles.emplace_back();
    sphere.ray.set(sphere_ray);
    sphere.center = Point{0, 3.f};
  };
  std::vector<float> start{to_rad(45), to_rad(-5), to_rad(-10)};
  std::vector<float> target{to_rad(135), to_rad(5), to_rad(10)};

  auto traj = connector.getTrajectory(start, target);
  EXPECT_EQ(traj->getState().convert(), start);

  auto traj_status = traj->advance();
  EXPECT_EQ(traj_status, Trajectory::AdvanceInfo::advanced);

  auto reached = traj->getState().convert();
  EXPECT_TRUE(reached[0] < to_rad(90));

  // check all joints are in between start and target
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_TRUE(reached[k] > start[k]);
    EXPECT_TRUE(reached[k] < target[k]);
  }

  {
    SCOPED_TRACE("try an additional steer");
    auto traj_status = traj->advance();
    EXPECT_EQ(traj_status, Trajectory::AdvanceInfo::advanced);

    auto reached2 = traj->getState().convert();
    EXPECT_TRUE(reached2[0] < to_rad(90));

    // check all joints are in between start and target
    for (std::size_t k = 0; k < 3; ++k) {
      EXPECT_TRUE(reached2[k] > reached[k]);
      EXPECT_TRUE(reached2[k] < target[k]);
    }
  }
}

TEST_F(PlanarRobotsTestBubbleAdvance, expect_blocked) {
  {
    auto &sphere = scene.obstacles.emplace_back();
    sphere.ray.set(sphere_ray);
    sphere.center = Point{robot_capsules_ray + sphere_ray + 0.00001f, 3.f};
  };
  std::vector<float> start{to_rad(90), 0, 0};
  std::vector<float> target{to_rad(135), to_rad(5), to_rad(10)};

  auto traj = connector.getTrajectory(start, target);

  auto traj_status = traj->advance();
  EXPECT_EQ(traj_status, Trajectory::AdvanceInfo::blocked);
  EXPECT_EQ(traj->getState().convert(), start);
}

TEST_F(PlanarRobotsTestBubbleAdvance, target_reached_no_obstacles) {
  std::vector<float> start{to_rad(45), to_rad(-5), to_rad(-10)};
  std::vector<float> target{to_rad(135), to_rad(5), to_rad(10)};

  auto traj = connector.getTrajectory(start, target);

  auto traj_status = traj->advance();
  EXPECT_EQ(traj_status, Trajectory::AdvanceInfo::targetReached);
  EXPECT_EQ(traj->getState().convert(), target);
}

TEST_F(PlanarRobotsTestBubbleAdvance, target_reached_in_1_steer) {
  {
    auto &sphere = scene.obstacles.emplace_back();
    sphere.ray.set(sphere_ray);
    sphere.center = Point{0, 10.f};
  };
  std::vector<float> start{to_rad(45), to_rad(-5), to_rad(-10)};
  std::vector<float> target{to_rad(135), to_rad(5), to_rad(10)};

  auto traj = connector.getTrajectory(start, target);

  auto traj_status = traj->advance();
  EXPECT_EQ(traj_status, Trajectory::AdvanceInfo::targetReached);
  EXPECT_EQ(traj->getState().convert(), target);
}

TEST_F(PlanarRobotsTestBubbleAdvance, target_reached_in_multiple_steer) {
  {
    auto &sphere = scene.obstacles.emplace_back();
    sphere.ray.set(sphere_ray);
    sphere.center = Point{0, 3.f + robot_capsules_ray + sphere_ray + 0.1f};
  };
  std::vector<float> start{to_rad(45), to_rad(-5), to_rad(-10)};
  std::vector<float> target{to_rad(135), to_rad(5), to_rad(10)};

  auto traj = connector.getTrajectory(start, target);

  std::size_t iter = 0;
  std::vector<std::vector<float>> intermediate_states;
  while (iter < 100) {
    auto traj_status = traj->advance();
    ASSERT_TRUE(traj_status != Trajectory::AdvanceInfo::blocked);
    if (traj_status == Trajectory::AdvanceInfo::targetReached) {
      break;
    }
    intermediate_states.emplace_back(traj->getState().convert());
  }

  // check that distance to target monotonically decreases iteration after
  // iteration
  std::vector<float> previous_distance_to_target;
  for (std::size_t k = 0; k < target.size(); ++k) {
    previous_distance_to_target.push_back(std::numeric_limits<float>::max());
  }
  for (const auto &intermediate_state : intermediate_states) {
    std::vector<float> distance_to_target;
    for (std::size_t k = 0; k < target.size(); ++k) {
      distance_to_target.push_back(std::abs(intermediate_state[k] - target[k]));
      EXPECT_TRUE(distance_to_target[k] <= previous_distance_to_target[k]);
    }
    previous_distance_to_target = std::move(distance_to_target);
  }

  EXPECT_EQ(traj->getState().convert(), target);
}
