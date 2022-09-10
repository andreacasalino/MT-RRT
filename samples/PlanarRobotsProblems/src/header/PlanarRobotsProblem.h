/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/PlanningProblem.h>

#include <Geometry.h>

#include <array>

namespace mt_rrt::samples {
using Point = utils::Point;
using PointPtr = std::shared_ptr<Point>;

using Sphere = utils::Sphere;
using Spheres = std::vector<Sphere>;

using Segment = std::array<PointPtr, 2>;
struct Capsule {
  Positive<float> ray;
  Segment segment;
};

float distance_capsule_sphere(const Capsule &cap, const Sphere &sphere);

float distance_capsules(const Capsule &cap_a, const Capsule &cap_b);

class JointLimits {
public:
  JointLimits() : JointLimits{-utils::PI, utils::PI} {};
  JointLimits(float min, float max);

  float min() const { return min_; }
  float max() const { return max_; }

private:
  float min_;
  float max_;
};

class Robot {
public:
  struct Link {
    Positive<float> ray = Positive<float>{0.1f};
    Positive<float> length = Positive<float>{1.f};
    JointLimits joint_limits;
  };
  std::vector<Link> getLinks() const;

  struct Base {
    Point position = {0, 0};
    float angle = 0;
  };
  Base getBase() const {
    return Base{*body.front().segment.at(0), base_angle};
  };

  Robot() : Robot{std::vector<Link>{Link{}}, Base{}} {};

  Robot(const std::vector<Link> &links, const Base &base);

  Robot(const Robot &o);
  Robot &operator=(const Robot &o);

  Robot(Robot &&) = default;
  Robot &operator=(Robot &&) = default;

  const std::vector<JointLimits> &getLimits() const { return limits; }

  std::size_t getJointsNumber() const { return links_lengths.size(); }
  const std::vector<Capsule> &getBody() const { return body; }

  void setPose(const State &jointsAngles);

  float distance(const Spheres &obstacles) const;
  float distance(const Robot &o) const;

  float delta(const State &start, const State &target);

private:
  std::vector<float> links_lengths;
  std::vector<Capsule> body;
  float base_angle;

  std::vector<JointLimits> limits;
};

struct Scene {
  std::vector<Robot> robots;
  Spheres obstacles;
};
using ScenePtr = std::shared_ptr<Scene>;

std::size_t total_dof(const std::vector<Robot> &robots);

class PosesConnector : public Connector {
public:
  PosesConnector() = default;
  PosesConnector(const PosesConnector &o) {
    scene = std::make_shared<Scene>(*o.scene);
  }

  ScenePtr scene = std::make_shared<Scene>();

  ConnectorPtr copy() const override {
    return std::make_unique<PosesConnector>(*this);
  }
  float minCost2Go(const State &start, const State &end) const override;
  TrajectoryPtr getTrajectory(const State &start,
                              const State &end) const override;
};

std::shared_ptr<ProblemDescription>
make_problem_description(const std::optional<Seed> &seed, const Scene &scene);
} // namespace mt_rrt::samples
