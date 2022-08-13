/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <TrivialProblem.h>

#include <array>
#include <optional>

namespace mt_rrt::samples {
static constexpr float PI = 3.1415926535f;

float to_rad(float angle);

using Point2D = std::array<float, 2>;

struct Box2D {
  Point2D min_corner;
  Point2D max_corner;
};

using Boxes2D = std::vector<Box2D>;

void traslate(Point2D &subject, const Point2D &traslation);
void traslate(Box2D &subject, const Point2D &traslation);
void traslate(Boxes2D &subject, const Point2D &traslation);

class Rotator {
public:
  Rotator(float angle);
  Rotator(float angle, const Point2D &rotation_center);

  void rotate(Point2D &subject) const;
  void rotate(Box2D &subject) const;
  void rotate(Boxes2D &subject) const;

private:
  const float cos_angle;
  const float sin_angle;

  struct RotationCenterInfo {
    Point2D pre_rotation_traslation;
    Point2D post_rotation_traslation;
  };
  std::optional<RotationCenterInfo> rotation_center;
};

class TrivialProblem2DConnector : public TrivialProblemConnector {
public:
  TrivialProblem2DConnector(const Boxes2D &obstacles);
};

std::shared_ptr<ProblemDescription>
make_trivial_problem_2D_description(const Boxes2D &obstacles);
} // namespace mt_rrt::samples
