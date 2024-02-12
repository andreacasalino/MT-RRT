/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Geometry.h>

#include <optional>

namespace mt_rrt::geom {
class Transform {
public:
  Transform(const std::optional<float> &rotation_angle,
            const std::optional<Point> &traslation);

  float getAngle() const {
    return atan2f(rotation.sin_angle, rotation.cos_angle);
  }
  const auto &getTraslation() const { return traslation; };

  static Transform combine(const Transform &pre, const Transform &post);

  static Transform rotationAroundCenter(float rotation_angle,
                                        const Point &center);

  Point seenFromRelativeFrame(const Point &subject) const;

  Point dotRotationMatrix(const Point &subject) const;
  Point dotRotationMatrixTrasp(const Point &subject) const;

private:
  struct RotationInfo {
    float cos_angle;
    float sin_angle;
  };
  RotationInfo rotation;
  Point traslation;
};
} // namespace mt_rrt::geom
