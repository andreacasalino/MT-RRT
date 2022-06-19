/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "TrivialProblem.h"

#include <math.h>

namespace mt_rrt::sample {
float to_rad(float angle) { return PI * angle / 180.f; }

Point2D &traslate(Point2D &subject, const Point2D &traslation) {
  subject[0] += traslation[0];
  subject[1] += traslation[1];
  return subject;
}

Box2D &traslate(Box2D &subject, const Point2D &traslation) {
  traslate(subject.min_corner, traslation);
  traslate(subject.max_corner, traslation);
  return subject;
}

Boxes2D &traslate(Boxes2D &subject, const Point2D &traslation) {
  for (auto &box : subject) {
    traslate(box, traslation);
  }
  return subject;
}

Rotator::Rotator(float angle) : cos_angle(cos(angle)), sin_angle(sin(angle)) {}

Rotator::Rotator(float angle, const Point2D rotation_center) : Rotator(angle) {
  auto &center = this->rotation_center.emplace();
  center.post_rotation_traslation = rotation_center;
  center.pre_rotation_traslation = {-rotation_center[0], -rotation_center[1]};
}

Point2D &Rotator::rotate(Point2D &subject) const {
  if (rotation_center) {
    traslate(subject, rotation_center->pre_rotation_traslation);
  }
  float new_x = cos_angle * subject[0] - sin_angle * subject[1];
  float new_y = sin_angle * subject[0] + cos_angle * subject[1];
  subject[0] = new_x;
  subject[1] = new_y;
  if (rotation_center) {
    traslate(subject, rotation_center->post_rotation_traslation);
  }
  return subject;
}

Box2D &Rotator::rotate(Box2D &subject) const {
  rotate(subject.min_corner);
  rotate(subject.max_corner);
  return subject;
}

Boxes2D &Rotator::rotate(Boxes2D &subject) const {
  for (auto &box : subject) {
    rotate(box);
  }
  return subject;
}

namespace {
utils::Box convert(const Box2D &subject) {
  State min_converted = State{subject.min_corner[0], subject.min_corner[1]};
  State max_converted = State{subject.max_corner[0], subject.max_corner[1]};
  return utils::Box{min_converted, max_converted};
}

utils::Boxes convert(const Boxes2D &subject) {
  utils::Boxes result;
  for (const auto &box : subject) {
    result.emplace_back(convert(box));
  }
  return result;
}
} // namespace

TrivialProblem::TrivialProblem(const Boxes2D &obstacles)
    : utils::PointConnector(convert(obstacles)) {}

std::shared_ptr<ProblemDescription>
make_trivial_problem(const Boxes2D &obstacles) {
  std::unique_ptr<TrivialProblem> connector =
      std::make_unique<TrivialProblem>(obstacles);

  State min_corner, max_corner;
  min_corner.reserve(connector->getStateSpaceSize());
  max_corner.reserve(connector->getStateSpaceSize());
  for (std::size_t k = 0; k < connector->getStateSpaceSize(); ++k) {
    min_corner.push_back(-1.f);
    max_corner.push_back(1.f);
  }
  std::unique_ptr<HyperBox> sampler =
      std::make_unique<HyperBox>(min_corner, max_corner);

  std::shared_ptr<ProblemDescription> result;
  result.reset(new ProblemDescription{std::move(sampler), std::move(connector),
                                      true, Positive<float>{10.f}});
  return result;
}
} // namespace mt_rrt::sample
