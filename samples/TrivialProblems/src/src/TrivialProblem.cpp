/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <TrivialProblem.h>

#include <MT-RRT-carpet/Error.h>

#include <algorithm>
#include <math.h>

namespace mt_rrt::samples {
float to_rad(float angle) { return angle * PI / 180.f; }

float to_grad(float angle) { return angle * 180.f / PI; }

Transform::Transform(const std::optional<float> &rotation_angle,
                     const std::optional<Traslation> &trsl)
    : traslation({0, 0}) {
  float angle = rotation_angle ? rotation_angle.value() : 0;
  rotation = RotationInfo{cosf(angle), sinf(angle)};
  if (trsl) {
    traslation = trsl.value();
  }
}

void Transform::dotRotationMatrix(float *recipient, const float *point) const {
  recipient[0] = rotation.cos_angle * point[0] - rotation.sin_angle * point[1];
  recipient[1] = rotation.sin_angle * point[0] + rotation.cos_angle * point[1];
}

void Transform::dotRotationMatrixTrasp(float *recipient,
                                       const float *point) const {
  recipient[0] = rotation.cos_angle * point[0] + rotation.sin_angle * point[1];
  recipient[1] = -rotation.sin_angle * point[0] + rotation.cos_angle * point[1];
}

State Transform::seenFromRelativeFrame(const State &subject) const {
  State delta = subject;
  delta[0] -= traslation[0];
  delta[1] -= traslation[1];
  State result = {0, 0};
  dotRotationMatrixTrasp(result.data(), delta.data());
  return result;
}

Transform Transform::combine(const Transform &pre, const Transform &post) {
  const auto &[pre_cos, pre_sin] = pre.rotation;
  const auto &[post_cos, post_sin] = post.rotation;

  Transform result(std::nullopt, std::nullopt);
  result.rotation = RotationInfo{pre_cos * post_cos - pre_sin * post_sin,
                                 pre_sin * post_cos + pre_cos * post_sin};

  pre.dotRotationMatrix(result.traslation.data(), post.traslation.data());
  result.traslation[0] += pre.traslation[0];
  result.traslation[1] += pre.traslation[1];
  return result;
}

Transform Transform::rotationAroundCenter(float rotation_angle,
                                          const Traslation &center) {
  Transform result(rotation_angle, center);
  result.dotRotationMatrix(result.traslation.data(), center.data());
  result.traslation[0] = center[0] - result.traslation[0];
  result.traslation[1] = center[1] - result.traslation[1];
  return result;
}

Box::Box(const State &min, const State &max,
         const std::optional<Transform> &trsf)
    : min_corner(min), max_corner(max), trsf(trsf) {
  if (min_corner[0] > max_corner[0]) {
    throw Error{"Invalid corners"};
  }
  if (min_corner[1] > max_corner[1]) {
    throw Error{"Invalid corners"};
  }
}

namespace {
static const float STATE_BOX_DIAGONAL_LENGTH = 2.f * sqrtf(1.f);
}

const float TrivialProblemConnector::STEER_DEGREE =
    STATE_BOX_DIAGONAL_LENGTH / 15.f;

TrivialProblemConnector::TrivialProblemConnector(const Boxes &obstacles)
    : TunneledConnector(2) {
  this->obstacles = std::make_shared<Boxes>(obstacles);
  setSteerDegree(STEER_DEGREE);
}

TrivialProblemConnector::TrivialProblemConnector(
    const TrivialProblemConnector &o)
    : TunneledConnector(o.getStateSpaceSize()) {
  this->obstacles = std::make_shared<Boxes>(*o.obstacles);
  setSteerDegree(STEER_DEGREE);
}

namespace {
enum class IntervalsCheck { Disjointed, EntirelyContained, Overlapping };
IntervalsCheck check_intervals(const State &segment_start,
                               const State &segment_end,
                               const State &min_corner, const State &max_corner,
                               const std::size_t pos) {
  const float segment_min = std::min(segment_start[pos], segment_end[pos]);
  const float segment_max = std::max(segment_start[pos], segment_end[pos]);
  if ((segment_max < min_corner[pos]) || (max_corner[pos] < segment_min)) {
    return IntervalsCheck::Disjointed;
  }
  if ((min_corner[pos] <= segment_min) && (segment_max <= max_corner[pos])) {
    return IntervalsCheck::EntirelyContained;
  }
  return IntervalsCheck::Overlapping;
}

bool collides(const State &segment_start, const State &segment_end,
              const State &min_corner, const State &max_corner) {
  float segment_l1_norm = 0;
  bool all_entirely_contained = true;
  for (std::size_t k = 0; k < segment_start.size(); ++k) {
    switch (check_intervals(segment_start, segment_end, min_corner, max_corner,
                            k)) {
    case IntervalsCheck::Disjointed:
      return false;
    case IntervalsCheck::Overlapping:
      all_entirely_contained = false;
      break;
    default:
      break;
    }
    float l1_norm = fabs(segment_start[k] - segment_end[k]);
    if (l1_norm > segment_l1_norm) {
      segment_l1_norm = l1_norm;
    }
  }

  if (all_entirely_contained) {
    return true;
  }

  if (segment_l1_norm < 1e-6f) {
    return true;
  }

  State mid_point;
  mid_point.reserve(segment_start.size());
  for (std::size_t k = 0; k < segment_start.size(); ++k) {
    mid_point.push_back(0.5f * (segment_start[k] + segment_end[k]));
  }
  return collides(segment_start, mid_point, min_corner, max_corner) ||
         collides(mid_point, segment_end, min_corner, max_corner);
}
} // namespace

bool collides(const State &segment_start, const State &segment_end,
              const Box &box) {
  if (box.trsf) {
    const auto segment_start_t = box.trsf->seenFromRelativeFrame(segment_start);
    const auto segment_end_t = box.trsf->seenFromRelativeFrame(segment_end);
    return collides(segment_start_t, segment_end_t, box.min_corner,
                    box.max_corner);
  }
  return collides(segment_start, segment_end, box.min_corner, box.max_corner);
}

bool TrivialProblemConnector::checkAdvancement(
    const State &previous_state, const State &advanced_state) const {
  const auto &boxes = *this->obstacles;
  if (boxes.empty()) {
    return false;
  }
  return std::any_of(boxes.begin(), boxes.end(), [&](const Box &obstacle) {
    return collides(previous_state, advanced_state, obstacle);
  });
}

std::shared_ptr<ProblemDescription>
make_trivial_problem_description(const std::optional<Seed> &seed,
                                 const Boxes &obstacles) {
  auto connector = std::make_unique<TrivialProblemConnector>(obstacles);

  State min_corner, max_corner;
  for (std::size_t k = 0; k < 2; ++k) {
    min_corner.push_back(-1.f);
    max_corner.push_back(1.f);
  }
  auto sampler = std::make_unique<HyperBox>(min_corner, max_corner, seed);

  std::shared_ptr<ProblemDescription> result;
  result.reset(new ProblemDescription{std::move(sampler), std::move(connector),
                                      true, Positive<float>{10.f}});
  return result;
}
} // namespace mt_rrt::samples
