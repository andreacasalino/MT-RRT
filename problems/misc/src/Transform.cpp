/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Transform.h>

namespace mt_rrt::geom {
Transform::Transform(float angle, PointAllocated tr) {
  rotation = RotationInfo{cosf(angle), sinf(angle)};
  traslation = tr;
}

PointAllocated Transform::dotRotationMatrix(const Point &subject) const {
  float x = rotation.cos_angle * subject.data()[0] -
            rotation.sin_angle * subject.data()[1];
  float y = rotation.sin_angle * subject.data()[0] +
            rotation.cos_angle * subject.data()[1];
  return PointAllocated{x, y};
}

PointAllocated Transform::dotRotationMatrixTrasp(const Point &subject) const {
  float x = rotation.cos_angle * subject.data()[0] +
            rotation.sin_angle * subject.data()[1];
  float y = -rotation.sin_angle * subject.data()[0] +
            rotation.cos_angle * subject.data()[1];
  return PointAllocated{x, y};
}

PointAllocated Transform::seenFromRelativeFrame(const Point &subject) const {
  auto delta = diff(subject, traslation);
  auto result = dotRotationMatrixTrasp(delta);
  return result;
}

Transform Transform::combine(const Transform &pre, const Transform &post) {
  const auto &[pre_cos, pre_sin] = pre.rotation;
  const auto &[post_cos, post_sin] = post.rotation;

  auto trsl = pre.dotRotationMatrix(post.traslation);
  float tx = trsl.x() + pre.traslation.x();
  float ty = trsl.y() + pre.traslation.y();

  Transform res;
  res.rotation = RotationInfo{pre_cos * post_cos - pre_sin * post_sin,
                              pre_sin * post_cos + pre_cos * post_sin};
  res.traslation = PointAllocated{tx, ty};
  return res;
}

Transform Transform::rotationAroundCenter(float rotation_angle,
                                          const Point &center) {
  Transform result(TransformBuilder{}.angle(rotation_angle));
  float x = (1.f - result.rotation.cos_angle) * center.data()[0] +
            result.rotation.sin_angle * center.data()[1];
  float y = -result.rotation.sin_angle * center.data()[0] +
            (1.f - result.rotation.cos_angle) * center.data()[1];
  result.traslation = PointAllocated{x, y};
  return result;
}
} // namespace mt_rrt::geom
