/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Transform.h>

namespace mt_rrt::geom {
Transform::Transform(const std::optional<float> &rotation_angle,
                     const std::optional<Point> &trsl) {
  float angle = rotation_angle ? rotation_angle.value() : 0;
  rotation = RotationInfo{cosf(angle), sinf(angle)};
  if (trsl) {
    traslation = trsl.value();
  }
}

Point Transform::dotRotationMatrix(const Point &subject) const {
  float x = rotation.cos_angle * subject.data()[0] -
            rotation.sin_angle * subject.data()[1];
  float y = rotation.sin_angle * subject.data()[0] +
            rotation.cos_angle * subject.data()[1];
  return Point{x, y};
}

Point Transform::dotRotationMatrixTrasp(const Point &subject) const {
  float x = rotation.cos_angle * subject.data()[0] +
            rotation.sin_angle * subject.data()[1];
  float y = -rotation.sin_angle * subject.data()[0] +
            rotation.cos_angle * subject.data()[1];
  return Point{x, y};
}

Point Transform::seenFromRelativeFrame(const Point &subject) const {
  auto delta = diff(subject, traslation);
  auto result = dotRotationMatrixTrasp(delta);
  return result;
}

Transform Transform::combine(const Transform &pre, const Transform &post) {
  const auto &[pre_cos, pre_sin] = pre.rotation;
  const auto &[post_cos, post_sin] = post.rotation;

  auto rot = RotationInfo{pre_cos * post_cos - pre_sin * post_sin,
                          pre_sin * post_cos + pre_cos * post_sin};

  auto trsl = pre.dotRotationMatrix(post.traslation);
  float tx = trsl.data()[0] + pre.traslation.data()[0];
  float ty = trsl.data()[1] + pre.traslation.data()[1];

  Transform result(std::nullopt, std::nullopt);
  result.rotation = rot;
  result.traslation = Point{tx, ty};
  return result;
}

Transform Transform::rotationAroundCenter(float rotation_angle,
                                          const Point &center) {
  Transform result(rotation_angle, std::nullopt);
  float x = (1.f - result.rotation.cos_angle) * center.data()[0] +
            result.rotation.sin_angle * center.data()[1];
  float y = -result.rotation.sin_angle * center.data()[0] +
            (1.f - result.rotation.cos_angle) * center.data()[1];
  result.traslation = Point{x, y};
  return result;
}
} // namespace mt_rrt::geom
