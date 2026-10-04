/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Error.h>

#include <Primitives.h>

namespace mt_rrt::geom {
Versor::Versor(float angle) : cos_sin{cosf(angle), sinf(angle)} {}

Versor::Versor(const Point &vector_start, const Point &vector_end)
    : Versor(atan2f(vector_end.y() - vector_start.y(),
                    vector_end.x() - vector_start.x())) {}

Segment::Segment(const Point &s, const Point &e)
    : start(PointAllocated::clone(s)), end{PointAllocated::clone(e)} {
  end_start_diff = diff(end, start);
}

Segment::Segment(const Point &s, const Versor &direction)
    : start(PointAllocated::clone(s)), end{},
      end_start_diff{PointAllocated::clone(direction.asPoint())} {
  end = sum(start, end_start_diff);
}

float Segment::closest_on_line(const Point &point) const {
  const auto &b_a = end_start_diff;
  Point c_a = diff(point, getStart());
  return dot_product(c_a, b_a) / dot_product(b_a, b_a);
}

std::optional<std::array<float, 2>>
Segment::closest_between_lines(const Segment &segment_a,
                               const Segment &segment_b) {
  const Point &V1 = segment_a.getEndStartDiff();
  const Point &V2 = segment_b.getEndStartDiff();

  const auto V0 = diff(segment_a.getStart(), segment_b.getStart());
  const float m00 = dot_product(V1, V1);
  const float m11 = dot_product(V2, V2);
  const float m01 = -dot_product(V1, V2);
  const float c0 = -dot_product(V0, V1);
  const float c1 = dot_product(V0, V2);
  const float determinant = m00 * m11 - m01 * m01;
  if (std::abs(determinant) < 0.0001f) {
    return std::nullopt;
  }
  const float s_min = (c0 * m11 - m01 * c1) / determinant;
  const float t_min = (c1 - m01 * s_min) / m11;
  return std::array<float, 2>{s_min, t_min};
}

PointAllocated Segment::at(float coeff) const {
  const auto &delta = getEndStartDiff();
  return sum(getStart(), delta, coeff);
}

Box::Box(const Point &min, const Point &max)
    : min_corner(PointAllocated::clone(min)),
      max_corner(PointAllocated::clone(max)), trsf(trsf) {
  if (min_corner.x() > max_corner.x()) {
    throw Error{"Invalid corners"};
  }
  if (min_corner.y() > max_corner.y()) {
    throw Error{"Invalid corners"};
  }
}

namespace {
enum class IntervalsCheck { Disjointed, EntirelyContained, Overlapping };
IntervalsCheck check_intervals(const Point &segment_start,
                               const Point &segment_end,
                               const Point &min_corner, const Point &max_corner,
                               const std::size_t pos) {
  auto start_data = segment_start.data();
  auto end_data = segment_end.data();
  const float segment_min = std::min(start_data[pos], end_data[pos]);
  const float segment_max = std::max(start_data[pos], end_data[pos]);
  auto min_corner_data = min_corner.data();
  auto max_corner_data = max_corner.data();
  if ((segment_max < min_corner_data[pos]) ||
      (max_corner_data[pos] < segment_min)) {
    return IntervalsCheck::Disjointed;
  }
  if ((min_corner_data[pos] <= segment_min) &&
      (segment_max <= max_corner_data[pos])) {
    return IntervalsCheck::EntirelyContained;
  }
  return IntervalsCheck::Overlapping;
}

bool collides_(const Point &segment_start, const Point &segment_end,
               const Point &min_corner, const Point &max_corner) {
  float segment_l1_norm = 0;
  bool all_entirely_contained = true;
  for (std::size_t k = 0; k < 2; ++k) {
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
    float l1_norm = fabs(segment_start.data()[k] - segment_end.data()[k]);
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

  PointAllocated mid_point{0.5f * (segment_start.x() + segment_end.x()),
                           0.5f * (segment_start.y() + segment_end.y())};
  return collides_(segment_start, mid_point, min_corner, max_corner) ||
         collides_(mid_point, segment_end, min_corner, max_corner);
}
} // namespace

bool Box::collides(const Segment &segment) const {
  if (trsf) {
    const auto segment_start_t =
        trsf->seenFromRelativeFrame(segment.getStart());
    const auto segment_end_t = trsf->seenFromRelativeFrame(segment.getEnd());
    return collides_(segment_start_t, segment_end_t, min_corner, max_corner);
  }
  return collides_(segment.getStart(), segment.getEnd(), min_corner,
                   max_corner);
}

} // namespace mt_rrt::geom
