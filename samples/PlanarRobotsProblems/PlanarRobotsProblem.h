/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/PlanningProblem.h>
#include <array>

namespace mt_rrt::sample {
using Point = std::array<float, 2>;
using PointPtr = std::shared_ptr<Point>;

struct Sphere {
  Positive<float> ray;
  Point center;
};

using Spheres = std::vector<Sphere>;

struct Capsule {
  Positive<float> ray;
  std::array<PointPtr, 2> segment;
};

class Robot {
public:
  struct Link {
    Positive<float> ray;
    Positive<float> length;
  };

  Robot(const std::vector<Link> &links, const Point &base);
  Robot(const Robot &o);

  std::size_t getJointsNumber() const { return links_lengths.size(); }
  const std::vector<Capsule> &getBody() const { return body; }

  void setPose(const State &jointsAngles);

  float distance(const Spheres &obstacles) const;
  float distance(const Robot &o) const;

  float delta(const State &start, const State &target);

private:
  const std::vector<float> links_lengths;
  std::vector<Capsule> body;
};

struct Scene {
  std::vector<Robot> robots;
  Spheres obstacles;
};
using ScenePtr = std::shared_ptr<Scene>;

class PosesConnector : public Connector {
public:
  PosesConnector() = default;

  ScenePtr scene = std::make_shared<Scene>();

  ConnectorPtr copy() const override {
    return std::make_unique<PosesConnector>(std::make_shared<Scene>(*scene));
  }
  float minCost2Go(const State &start, const State &end) const override;
  TrajectoryPtr getTrajectory(const State &start,
                              const State &end) const override;
};

// // TODO explain that a Connector and a Sampler are fundamentally created
// ProblemDescription make_problem_description();
} // namespace mt_rrt::sample
