/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/ProblemDescription.h>
#include <MT-RRT/TunneledConnector.h>
#include <MT-RRT/Types.h>

#include <Primitives.h>

namespace mt_rrt::robots {
namespace detail {
template <typename Iter, typename Pred>
float min_distance(Iter begin, Iter end, Pred &&pred) {
  float res = std::numeric_limits<float>::max(), tmp;
  for (auto it = begin; it != end; ++it) {
    tmp = pred(*it);
    if (tmp == 0)
      return 0;
    res = std::min<float>(res, tmp);
  }
  return res;
}
} // namespace detail

struct Capsule {
  const Positive<float> &ray;
  geom::Segment segment;
};

float distance_capsule_sphere(const Capsule &cap, const geom::Sphere &sphere);

float distance_capsules(const Capsule &cap_a, const Capsule &cap_b);

class JointLimits {
public:
  JointLimits() : JointLimits{-geom::PI, geom::PI} {};
  JointLimits(float min, float max);

  float min() const { return min_; }
  float max() const { return max_; }

private:
  float min_;
  float max_;
};

struct Joint {
  JointLimits limits;
  Positive<float> ray = Positive<float>{0.1f};
  Positive<float> length = Positive<float>{1.f};
};

class Robot {
public:
  struct Base {
    geom::Point position = geom::Point{0, 0};
    float angle = 0;
  };
  Base getBase() const {
    return Base{body.front().segment.getStart(), base_angle};
  };
  const auto &getJoints() const { return joints; }
  std::size_t getJointsNumber() const { return joints.size(); }
  const std::vector<Capsule> &getBody() const { return body; }

  Robot() : Robot{std::vector<Joint>{Joint{}}, Base{}} {};

  Robot(const std::vector<Joint> &joints) : Robot{joints, Base{}} {}
  Robot(const std::vector<Joint> &joints, const Base &base);

  Robot(const Robot &o);
  Robot &operator=(const Robot &o);

  Robot(Robot &&) = default;
  Robot &operator=(Robot &&) = default;

  void setPose(const View &jointsAngles);

  template <typename Iter> float distance(Iter begin, Iter end) const {
    return detail::min_distance(begin, end, [&](const geom::Sphere &obstacle) {
      return detail::min_distance(
          body.begin(), body.end(), [&](const Capsule &cap) {
            return distance_capsule_sphere(cap, obstacle);
          });
    });
  }

  float distance(const Robot &o) const;

  float delta(const View &start, const View &target);

private:
  std::vector<Joint> joints;
  float base_angle;
  std::vector<Capsule> body;
};

struct Scene {
  std::vector<Robot> robots;
  std::vector<geom::Sphere> obstacles;
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
  float minCost2Go(const View &start, const View &end) const override;
  TrajectoryPtr getTrajectory(const View &start,
                              const View &end) const override;

  static std::shared_ptr<ProblemDescription>
  make(const std::optional<Seed> &seed, const Scene &scene);
};
} // namespace mt_rrt::robots
