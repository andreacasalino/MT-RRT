/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <TrivialProblem2D.h>

#include <math.h>

namespace mt_rrt::samples {
float to_rad(float angle) { return PI * angle / 180.f; }

void traslate(Point2D &subject, const Point2D &traslation) {
  subject[0] += traslation[0];
  subject[1] += traslation[1];
}

void traslate(Box2D &subject, const Point2D &traslation) {
  traslate(subject.min_corner, traslation);
  traslate(subject.max_corner, traslation);
}

void traslate(Boxes2D &subject, const Point2D &traslation) {
  for (auto &box : subject) {
    traslate(box, traslation);
  }
}

Rotator::Rotator(float angle) : cos_angle(cos(angle)), sin_angle(sin(angle)) {}

Rotator::Rotator(float angle, const Point2D &rotation_center) : Rotator(angle) {
  auto &center = this->rotation_center.emplace();
  center.post_rotation_traslation = rotation_center;
  center.pre_rotation_traslation = {-rotation_center[0], -rotation_center[1]};
}

void Rotator::rotate(Point2D &subject) const {
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
}

void Rotator::rotate(Box2D &subject) const {
  rotate(subject.min_corner);
  rotate(subject.max_corner);
}

void Rotator::rotate(Boxes2D &subject) const {
  for (auto &box : subject) {
    rotate(box);
  }
}

namespace {
Box convert(const Box2D &subject) {
  State min_converted = State{subject.min_corner[0], subject.min_corner[1]};
  State max_converted = State{subject.max_corner[0], subject.max_corner[1]};
  return Box{min_converted, max_converted};
}

Boxes convert(const Boxes2D &subject) {
  Boxes result;
  for (const auto &box : subject) {
    result.emplace_back(convert(box));
  }
  return result;
}
} // namespace

TrivialProblem2DConnector::TrivialProblem2DConnector(const Boxes2D &obstacles)
    : TrivialProblemConnector(convert(obstacles)) {}

std::shared_ptr<ProblemDescription>
make_trivial_problem_2D_description(const Boxes2D &obstacles) {
  std::unique_ptr<TrivialProblem2DConnector> connector =
      std::make_unique<TrivialProblem2DConnector>(obstacles);

  std::shared_ptr<ProblemDescription> result;
  result.reset(new ProblemDescription{
      make_trivial_problem_sampler(connector->getStateSpaceSize(),
                                   std::nullopt),
      std::move(connector), true, Positive<float>{10.f}});
  return result;
}
} // namespace mt_rrt::samples
