/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <PlanarRobotsProblem.h>

#include <algorithm>
#include <array>
#include <cmath>

namespace mt_rrt::robots {
float distance_capsule_sphere(const Capsule &cap, const geom::Sphere &sphere) {
  float result;
  if (float s_min = cap.segment.closest_on_line(sphere.center); s_min < 0) {
    result = geom::distance(sphere.center, cap.segment.getStart());
  } else if (s_min > 1.f) {
    result = geom::distance(sphere.center, cap.segment.getEnd());
  } else {
    result = geom::distance(sphere.center, cap.segment.at(s_min));
  }
  result -= cap.ray.get() + sphere.ray.get();
  result = std::max<float>(result, 0.f);
  return result;
}

float distance_capsules(const Capsule &cap_a, const Capsule &cap_b) {
  if (auto pair = geom::Segment::closest_on_lines(cap_a.segment, cap_b.segment);
      pair.has_value()) {
    const float s_min = pair->front();
    const float t_min = pair->back();
    if ((s_min >= 0) && (s_min <= 1.f) && (t_min >= 0) && (t_min <= 1.f)) {
      float result =
          geom::distance(cap_a.segment.at(s_min), cap_b.segment.at(t_min));
      result -= cap_a.ray.get() + cap_b.ray.get();
      result = std::max<float>(result, 0.f);
      return result;
    }
  }

  float res = std::numeric_limits<float>::max(), tmp;
  tmp = distance_capsule_sphere(
      cap_a, geom::Sphere{cap_b.ray, cap_b.segment.getStart()});
  if (tmp == 0)
    return 0;
  else
    res = std::min<float>(res, tmp);
  tmp = distance_capsule_sphere(
      cap_a, geom::Sphere{cap_b.ray, cap_b.segment.getEnd()});
  if (tmp == 0)
    return 0;
  else
    res = std::min<float>(res, tmp);
  tmp = distance_capsule_sphere(
      cap_b, geom::Sphere{cap_a.ray, cap_a.segment.getStart()});
  if (tmp == 0)
    return 0;
  else
    res = std::min<float>(res, tmp);
  tmp = distance_capsule_sphere(
      cap_b, geom::Sphere{cap_a.ray, cap_a.segment.getEnd()});
  if (tmp == 0)
    return 0;
  else
    res = std::min<float>(res, tmp);

  return res;
}

JointLimits::JointLimits(float min, float max) : min_(min), max_(max) {
  if (max < min) {
    throw Error{"Invalid joint limits"};
  }
}

Robot::Robot(const std::vector<Joint> &joints, const Base &base)
    : joints(joints), base_angle(base.angle) {
  if (joints.empty()) {
    throw Error{"A robot should be made of at least one link"};
  }
  std::vector<float> zeros;
  for (std::size_t k = 0; k < joints.size(); ++k) {
    zeros.push_back(0);
  }
  // set the base
  body.emplace_back(
      Capsule{joints.front().ray, geom::Segment{base.position, base.position}});
  setPose(View{zeros});
}

Robot::Robot(const Robot &o) : Robot{o.getJoints(), o.getBase()} {}

Robot &Robot::operator=(const Robot &o) {
  joints = o.joints;
  base_angle = o.base_angle;
  body.clear();
  for (std::size_t k = 0; k < body.size(); ++k) {
    body.emplace_back(Capsule{joints[k].ray, o.body[k].segment});
  }
  return *this;
}

namespace {
struct LinkGenerator {
  LinkGenerator(const geom::Point &base, float base_angle)
      : angleAbs{base_angle}, point{base} {};

  geom::Segment update(float len, float angle) {
    angleAbs += angle;
    geom::Point next{point.data()[0] + len * cosf(angleAbs),
                     point.data()[1] + len * sinf(angleAbs)};
    geom::Segment res{point, next};
    point = std::move(next);
    return res;
  }

private:
  float angleAbs;
  geom::Point point;
};

} // namespace

void Robot::setPose(const View &jointsAngles) {
  if (jointsAngles.size != joints.size()) {
    throw Error{"Invalid pose"};
  }
  LinkGenerator gen{body.front().segment.getStart(), base_angle};
  body.clear();
  for (std::size_t k = 0; k < joints.size(); ++k) {
    body.emplace_back(Capsule{joints[k].ray, gen.update(joints[k].length.get(),
                                                        jointsAngles.data[k])});
  }
}

float Robot::distance(const Robot &o) const {
  return detail::min_distance(
      body.begin(), body.end(), [&](const Capsule &this_cap) {
        const auto &o_body = o.getBody();
        return detail::min_distance(
            o_body.begin(), o_body.end(),
            [&this_cap = this_cap](const Capsule &o_cap) {
              return distance_capsules(this_cap, o_cap);
            });
      });
}

float Robot::delta(const View &start, const View &target) {
  setPose(start);

  auto compute_ray = [&body = this->body](std::size_t link) -> float {
    float result = 0;
    const auto &joint_center = body[link].segment.getStart();
    for (std::size_t l = link; l < body.size(); ++l) {
      const auto &link_body = body[l];
      const auto &end = link_body.segment.getEnd();
      float dist = geom::distance(end, joint_center) + link_body.ray.get();
      result = std::max<float>(result, dist);
    }
    return result;
  };

  float result = 0;
  for (std::size_t l = 0; l < body.size(); ++l) {
    result += std::abs(start.data[l] - target.data[l]) * compute_ray(l);
  }
  return result;
}

std::size_t total_dof(const std::vector<Robot> &robots) {
  std::size_t result = 0;
  for (const auto &robot : robots) {
    result += robot.getJointsNumber();
  }
  return result;
}

namespace {
struct ViewMutable {
  ViewMutable(const float *data, std::size_t size)
      : size{size}, data{const_cast<float *>(data)} {}

  operator View() { return View{data, size}; }

  std::size_t size;
  float *data;
};

template <typename ViewT> using Views = std::vector<ViewT>;

template <typename ViewT>
Views<ViewT> split_state(const View &state, const std::vector<Robot> &robots) {
  Views<ViewT> result;
  const float *data = state.data;
  for (const auto &robot : robots) {
    result.emplace_back(data, robot.getJointsNumber());
    data += robot.getJointsNumber();
  }
  return result;
}

class SceneTrajectory : public Trajectory {
public:
  SceneTrajectory(const ScenePtr &scene, const View &target_state,
                  const View &start_state)
      : scene(scene), start_state_(start_state),
        target_state_(split_state<View>(target_state, scene->robots)) {
    current_state_buffer = start_state.convert();
    current_state =
        split_state<ViewMutable>(View{current_state_buffer}, scene->robots);
  }

  AdvanceInfo advance() override;
  View getState() const override {
    return View{current_state_buffer.data(), current_state_buffer.size()};
  }
  float getCumulatedCost() const override {
    return geom::distance(start_state_, View{current_state_buffer});
  }

private:
  ScenePtr scene;

  View start_state_;
  const Views<View> target_state_;

  std::vector<float> current_state_buffer;
  Views<ViewMutable> current_state;
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

void advance_(float *subject, const float *target, std::size_t size,
              float advancement) {
  for (std::size_t k = 0; k < size; ++k) {
    subject[k] += advancement * (target[k] - subject[k]);
  }
}

Trajectory::AdvanceInfo SceneTrajectory::advance() {
  auto &robots = scene->robots;
  const auto &obstacles = scene->obstacles;

  std::vector<ProximityInfo> proximity_infos;

  std::vector<float> deltas;
  for (std::size_t j = 0; j < robots.size(); ++j) {
    auto &robot = robots[j];
    deltas.push_back(robot.delta(current_state[j], target_state_[j]));
    proximity_infos.emplace_back(ProximityInfo{
        robot.distance(obstacles.begin(), obstacles.end()), deltas.back()});
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
    for (std::size_t k = 0; k < current_state.size(); ++k) {
      std::memcpy(current_state[k].data, target_state_[k].data,
                  sizeof(float) * current_state[k].size);
    }
    return AdvanceInfo::targetReached;
  }
  // update state according to advancement
  for (std::size_t k = 0; k < current_state.size(); ++k) {
    advance_(current_state[k].data, target_state_[k].data,
             current_state[k].size, advancement);
  }
  return AdvanceInfo::advanced;
}
} // namespace

float PosesConnector::minCost2Go(const View &start, const View &end) const {
  return geom::distance(start, end);
}

TrajectoryPtr PosesConnector::getTrajectory(const View &start,
                                            const View &end) const {
  return std::make_unique<SceneTrajectory>(scene, end, start);
}

namespace {
static constexpr float GAMMA = 10.f;
}

std::shared_ptr<ProblemDescription>
PosesConnector::make(const std::optional<Seed> &seed, const Scene &scene) {
  std::unique_ptr<PosesConnector> connector =
      std::make_unique<PosesConnector>();
  *connector->scene = scene;

  std::vector<float> min_corner;
  std::vector<float> max_corner;
  for (const auto &robot : connector->scene->robots) {
    for (const auto &joint : robot.getJoints()) {
      min_corner.push_back(joint.limits.min());
      max_corner.push_back(joint.limits.max());
    }
  }

  std::shared_ptr<ProblemDescription> result;
  result.reset(new ProblemDescription{
      true, Positive<float>{GAMMA},
      std::make_unique<HyperBox>(min_corner, max_corner, seed),
      std::move(connector)});
  return result;
}
} // namespace mt_rrt::robots
