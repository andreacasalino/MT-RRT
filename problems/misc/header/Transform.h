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
struct TransformBuilder {
  TransformBuilder() = default;

  TransformBuilder &angle(float val) {
    angle_ = val;
    return *this;
  }

  TransformBuilder &traslation(const Point &val) {
    traslation_ = PointAllocated::clone(val);
    return *this;
  }

  float angle_{0};
  PointAllocated traslation_{};
};

static inline const TransformBuilder NULL_TRASNFORM = TransformBuilder{};

class Transform {
public:
  Transform() : Transform{NULL_TRASNFORM} {}
  Transform(const TransformBuilder &builder)
      : Transform{builder.angle_, builder.traslation_} {}

  Transform(const Transform &) = default;
  Transform &operator=(const Transform &) = default;
  Transform(Transform &&) noexcept = default;
  Transform &operator=(Transform &&) noexcept = default;

  [[nodiscard]] float getAngle() const {
    return atan2f(rotation.sin_angle, rotation.cos_angle);
  }
  const auto &getTraslation() const { return traslation; };

  [[nodiscard]] static Transform combine(const Transform &pre,
                                         const Transform &post);

  [[nodiscard]] static Transform rotationAroundCenter(float rotation_angle,
                                                      const Point &center);

  [[nodiscard]] PointAllocated
  seenFromRelativeFrame(const Point &subject) const;

  [[nodiscard]] PointAllocated dotRotationMatrix(const Point &subject) const;
  [[nodiscard]] PointAllocated
  dotRotationMatrixTrasp(const Point &subject) const;

private:
  Transform(float angle, PointAllocated tr);

  struct RotationInfo {
    float cos_angle;
    float sin_angle;
  };
  RotationInfo rotation;
  PointAllocated traslation;
};
} // namespace mt_rrt::geom
