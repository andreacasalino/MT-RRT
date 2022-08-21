/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "PlanarRobotsProblem.h"

#include <cmath>

namespace mt_rrt::samples {
float to_rad(float angle) { return PI * angle / 180.f; }

float euclidean_distance(const float *a, const float *b, std::size_t size) {
  float result = 0;
  for (std::size_t pos = 0; pos < size; ++pos) {
    result += powf(a[pos] - b[pos], 2.f);
  }
  return sqrtf(result);
}

namespace {
constexpr float MAX_DISTANCE = std::numeric_limits<float>::max();

float min_distance(const std::vector<float> &distances) {
  float result = MAX_DISTANCE;
  for (const auto &val : distances) {
    result = std::min(result, val);
  }
  return result;
}

// returns a - b
Point minus(const Point &a, const Point &b) {
  return {a.at(0) - b.at(0), a.at(1) - b.at(1)};
}

float dot(const Point &a, const Point &b) { return a[0] * b[0] + a[1] * b[1]; }

void advance_(float *subject, const float *target, std::size_t size,
              float advancement) {
  for (std::size_t k = 0; k < size; ++k) {
    subject[k] += advancement * (target[k] - subject[k]);
  }
}
} // namespace

float closest_point(const Segment &line, const Point &point) {
  const Point BA = minus(*line.back(), *line.front());
  const Point CA = minus(point, *line.front());
  // closest point to sphere center along line defined by A-B
  return dot(CA, BA) / dot(BA, BA);
}

std::optional<std::array<float, 2>> closest_pair(const Segment &line_a,
                                                 const Segment &line_b) {
  // TODO what happens if segments are parallel?
  const Point V0 = minus(*line_a[0], *line_b[0]);
  const Point V1 = minus(*line_a[1], *line_a[0]);
  const Point V2 = minus(*line_b[1], *line_b[0]);
  const float m00 = dot(V1, V1);
  const float m11 = dot(V2, V2);
  const float m01 = -dot(V1, V2);
  const float c0 = -dot(V0, V1);
  const float c1 = dot(V0, V2);
  const float determinant = m00 * m11 - m01 * m01;
  if (std::abs(determinant) < 0.0001f) {
    return std::nullopt;
  }
  const float s_min = (c0 * m11 - m01 * c1) / determinant;
  const float t_min = (c1 - m01 * s_min) / m11;
  return std::array<float, 2>{s_min, t_min};
}

float distance_capsule_sphere(const Capsule &cap, const Sphere &sphere) {
  float s_min = closest_point(cap.segment, sphere.center);
  float result;
  if (s_min < 0) {
    result = euclidean_distance(sphere.center.data(),
                                cap.segment.front()->data(), 2);
  } else if (s_min > 1.f) {
    result =
        euclidean_distance(sphere.center.data(), cap.segment.back()->data(), 2);
  } else {
    Point closest = *cap.segment[0];
    advance_(closest.data(), cap.segment[1]->data(), 2, s_min);
    result = euclidean_distance(closest.data(), sphere.center.data(), 2);
  }
  result -= cap.ray.get() + sphere.ray.get();
  result = std::max(result, 0.f);
  return result;
}

float distance_capsules(const Capsule &cap_a, const Capsule &cap_b) {
  const auto pair = closest_pair(cap_a.segment, cap_b.segment);
  if (pair) {
    const float s_min = pair->front();
    const float t_min = pair->back();
    if ((s_min >= 0) && (s_min <= 1.f) && (t_min >= 0) && (t_min <= 1.f)) {
      Point closest_in_cap_a = *cap_a.segment[0];
      advance_(closest_in_cap_a.data(), cap_a.segment[1]->data(), 2, s_min);
      Point closest_in_cap_b = *cap_a.segment[0];
      advance_(closest_in_cap_b.data(), cap_b.segment[1]->data(), 2, t_min);
      float result = euclidean_distance(closest_in_cap_a.data(),
                                        closest_in_cap_b.data(), 2);
      result -= cap_a.ray.get() + cap_b.ray.get();
      result = std::max(result, 0.f);
      return result;
    }
  }
  std::vector<float> distances;
  distances.push_back(
      distance_capsule_sphere(cap_a, Sphere{cap_b.ray, *cap_b.segment[0]}));
  distances.push_back(
      distance_capsule_sphere(cap_a, Sphere{cap_b.ray, *cap_b.segment[1]}));
  distances.push_back(
      distance_capsule_sphere(cap_b, Sphere{cap_a.ray, *cap_a.segment[0]}));
  distances.push_back(
      distance_capsule_sphere(cap_b, Sphere{cap_a.ray, *cap_a.segment[1]}));
  return min_distance(distances);
}

JointLimits::JointLimits(float min, float max) : min_(min), max_(max) {
  if (max < min) {
    throw Error{"Invalid joint limits"};
  }
}

namespace {
std::vector<float> compute_lengths(const std::vector<Robot::Link> &links) {
  std::vector<float> result;
  for (const auto &link : links) {
    result.push_back(link.length.get());
  }
  return result;
}
} // namespace

Robot::Robot(const std::vector<Link> &links, const Base &base)
    : links_lengths(compute_lengths(links)), base_angle(base.angle) {
  if (links_lengths.empty()) {
    throw Error{"A robot should be made of at least one link"};
  }
  PointPtr link_initial_point = std::make_shared<Point>(base.position);
  for (const auto &link : links) {
    link_initial_point = body.emplace_back(Capsule{link.ray,
                                                   {link_initial_point,
                                                    std::make_shared<Point>()}})
                             .segment.at(1);
    limits.emplace_back(link.joint_limits);
  }
  State zeros;
  for (std::size_t k = 0; k < links.size(); ++k) {
    zeros.push_back(0);
  }
  setPose(zeros);
}

std::vector<Robot::Link> Robot::getLinks() const {
  const auto &body = getBody();
  const auto &limits = getLimits();

  std::vector<Robot::Link> result;
  for (std::size_t pos = 0; pos < body.size(); ++pos) {
    auto &element = result.emplace_back();

    auto &link = body[pos];
    element.ray.set(link.ray.get());
    element.length.set(euclidean_distance(link.segment.front()->data(),
                                          link.segment.back()->data(), 2));
    element.joint_limits = limits[pos];
  }
  return result;
}

Robot::Robot(const Robot &o) : Robot{o.getLinks(), o.getBase()} {}

Robot &Robot::operator=(const Robot &o) {
  Robot copy{o};
  links_lengths = copy.links_lengths;
  body = copy.body;
  limits = copy.limits;
  return *this;
}

void Robot::setPose(const State &jointsAngles) {
  if (jointsAngles.size() != links_lengths.size()) {
    throw Error{"Invalid pose"};
  }
  float absolute_angle = base_angle;
  Point *link_initial_point = body.front().segment[0].get();
  for (std::size_t k = 0; k < links_lengths.size(); ++k) {
    absolute_angle += jointsAngles[k];
    body[k].segment[1]->at(0) =
        link_initial_point->at(0) + links_lengths[k] * cosf(absolute_angle);
    body[k].segment[1]->at(1) =
        link_initial_point->at(1) + links_lengths[k] * sinf(absolute_angle);
    link_initial_point = body[k].segment[1].get();
  }
}

float Robot::distance(const Spheres &obstacles) const {
  std::vector<float> distances;
  for (const auto &obstacle : obstacles) {
    for (const auto &cap : body) {
      distances.push_back(distance_capsule_sphere(cap, obstacle));
    }
  }
  return min_distance(distances);
}

float Robot::distance(const Robot &o) const {
  std::vector<float> distances;
  for (const auto &shape_this : body) {
    for (const auto &shape_o : o.body) {
      distances.push_back(distance_capsules(shape_this, shape_o));
    }
  }
  return min_distance(distances);
}

float Robot::delta(const State &start, const State &target) {
  setPose(start);

  auto compute_ray = [&body = this->body](std::size_t link) -> float {
    float result = 0;
    const auto &joint_center = *body[link].segment[0];
    for (std::size_t l = link; l < body.size(); ++l) {
      const auto &link_body = body[l];
      const auto dist = euclidean_distance(link_body.segment[1]->data(),
                                           joint_center.data(), 2) +
                        link_body.ray.get();
      if (result < dist) {
        result = dist;
      }
    }
    return result;
  };

  float result = 0;
  for (std::size_t l = 0; l < body.size(); ++l) {
    result += std::abs(start[l] - target[l]) * compute_ray(l);
  }
  return result;
}

namespace {
using Poses = std::vector<State>;

Poses split_state(const State &state, const std::vector<Robot> &robots) {
  Poses result;
  std::size_t pos = 0;
  for (const auto &robot : robots) {
    const auto dof = robot.getJointsNumber();
    result.emplace_back(State{state.begin() + pos, state.begin() + pos + dof});
    pos += dof;
  }
  return result;
}

class SceneTrajectory : public Trajectory {
public:
  SceneTrajectory(const ScenePtr &scene, const State &target_state,
                  const State &start_state)
      : scene(scene), start_state_(start_state),
        target_state_(split_state(target_state, scene->robots)) {
    current_state = split_state(start_state_, scene->robots);
  }

  AdvanceInfo advance() override;
  State getState() const override {
    State result;
    for (const auto &state : current_state) {
      result.insert(result.end(), state.begin(), state.end());
    }
    return result;
  }
  float getCumulatedCost() const override {
    // the below code is just for computing the euclidean distance from start to
    // current
    float result = 0;
    std::size_t pos = 0;
    for (std::size_t r = 0; r < current_state.size(); ++r) {
      result +=
          euclidean_distance(current_state[r].data(), start_state_.data() + pos,
                             current_state[r].size());
      pos += current_state[r].size();
    }
    return result;
  }

private:
  ScenePtr scene;

  const State start_state_;
  const Poses target_state_;

  Poses current_state;
};

constexpr float MIN_ADVANCEMENT = 0.01f;

struct ProximityInfo {
  float distance;
  float delta;
};
float compute_advancement(const std::vector<ProximityInfo> &values) {
  float advancement = 1.f;
  for (const auto &[distance, delta] : values) {
    if (delta > 0.0001f) {
      advancement = std::min(advancement, distance / delta);
    }
  }
  return std::max(advancement, 0.f);
}

AdvanceInfo SceneTrajectory::advance() {
  auto &robots = scene->robots;
  const auto &obstacles = scene->obstacles;

  std::vector<ProximityInfo> proximity_infos;

  std::vector<float> deltas;
  for (std::size_t j = 0; j < robots.size(); ++j) {
    auto &robot = robots[j];
    deltas.push_back(robot.delta(current_state[j], target_state_[j]));
    proximity_infos.emplace_back(
        ProximityInfo{robot.distance(obstacles), deltas.back()});
  }
  for (std::size_t j = 0; j < robots.size(); ++j) {
    for (std::size_t k = j + 1; k < robots.size(); ++k) {
      proximity_infos.emplace_back(
          ProximityInfo{robots[j].distance(robots[k]), deltas[j] + deltas[k]});
    }
  }

  auto advancement = compute_advancement(proximity_infos);
  if (advancement <= MIN_ADVANCEMENT) {
    return AdvanceInfo::blocked;
  }
  if (std::abs(advancement - 1.f) < MIN_ADVANCEMENT) {
    current_state = target_state_;
    return AdvanceInfo::targetReached;
  }
  // update state according to advancement
  for (std::size_t k = 0; k < current_state.size(); ++k) {
    advance_(current_state[k].data(), target_state_[k].data(),
             current_state[k].size(), advancement);
  }
  return AdvanceInfo::advanced;
}
} // namespace

float PosesConnector::minCost2Go(const State &start, const State &end) const {
  return euclidean_distance(start.data(), end.data(), start.size());
}

TrajectoryPtr PosesConnector::getTrajectory(const State &start,
                                            const State &end) const {
  return std::make_unique<SceneTrajectory>(scene, end, start);
}

namespace {
static constexpr float GAMMA = 10.f;

}

std::size_t total_dof(const std::vector<Robot> &robots) {
  std::size_t result = 0;
  for (const auto &robot : robots) {
    result += robot.getJointsNumber();
  }
  return result;
}

std::shared_ptr<ProblemDescription>
make_problem_description(const std::optional<Seed> &seed, const Scene &scene) {
  std::unique_ptr<PosesConnector> connector =
      std::make_unique<PosesConnector>();
  *connector->scene = scene;

  State min_corner;
  State max_corner;
  for (const auto &robot : connector->scene->robots) {
    for (const auto &limit : robot.getLimits()) {
      min_corner.push_back(limit.min());
      max_corner.push_back(limit.max());
    }
  }

  std::shared_ptr<ProblemDescription> result;
  result.reset(new ProblemDescription{
      std::make_unique<HyperBox>(min_corner, max_corner, seed),
      std::move(connector), true, Positive<float>{GAMMA}});
  return result;
}
} // namespace mt_rrt::samples