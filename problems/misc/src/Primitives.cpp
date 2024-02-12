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
    : Versor(atan2f(vector_end.data()[1] - vector_start.data()[1],
                    vector_end.data()[0] - vector_start.data()[0])) {}

Segment::Segment(const Point &start, const Point &end)
    : start(start), end{end} {
  end_start_diff = diff(end, start);
}

Segment::Segment(const Point &start, const Versor &direction) : start(start) {
  end_start_diff = direction.asPoint();
  end = sum(start, end_start_diff);
}

float Segment::closest_on_line(const Point &point) const {
  const auto &b_a = end_start_diff;
  Point c_a = diff(point, getStart());
  return dot(c_a, b_a) / dot(b_a, b_a);
}

std::optional<std::array<float, 2>>
Segment::closest_on_lines(const Segment &segment_a, const Segment &segment_b) {
  const Point &V1 = segment_a.getEndStartDiff();
  const Point &V2 = segment_b.getEndStartDiff();

  const Point V0 = diff(segment_a.getStart(), segment_b.getStart());
  const float m00 = dot(V1, V1);
  const float m11 = dot(V2, V2);
  const float m01 = -dot(V1, V2);
  const float c0 = -dot(V0, V1);
  const float c1 = dot(V0, V2);
  const float determinant = m00 * m11 - m01 * m01;
  if (std::abs(determinant) < 0.0001f) {
    return std::nullopt;
  }
  const float s_min = (c0 * m11 - m01 * c1) / determinant;
  const float t_min = (c1 - m01 * s_min) / m11;
  return std::array<float, 2>{s_min, t_min};
}

Point Segment::at(float coeff) const {
  const auto &delta = getEndStartDiff();
  return sum(getStart(), delta, coeff);
}

Box::Box(const Point &min, const Point &max,
         const std::optional<Transform> &trsf)
    : min_corner(min), max_corner(max), trsf(trsf) {
  if (min_corner.data()[0] > max_corner.data()[0]) {
    throw Error{"Invalid corners"};
  }
  if (min_corner.data()[1] > max_corner.data()[1]) {
    throw Error{"Invalid corners"};
  }
}

namespace {
enum class IntervalsCheck { Disjointed, EntirelyContained, Overlapping };
IntervalsCheck check_intervals(const float *segment_start,
                               const float *segment_end,
                               const float *min_corner, const float *max_corner,
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

bool collides_(const Point &segment_start, const Point &segment_end,
               const Point &min_corner, const Point &max_corner) {
  float segment_l1_norm = 0;
  bool all_entirely_contained = true;
  for (std::size_t k = 0; k < 2; ++k) {
    switch (check_intervals(segment_start.data(), segment_end.data(),
                            min_corner.data(), max_corner.data(), k)) {
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

  Point mid_point{0.5f * (segment_start.data()[0] + segment_end.data()[0]),
                  0.5f * (segment_start.data()[1] + segment_end.data()[1])};
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
