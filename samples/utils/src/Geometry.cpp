/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Geometry.h>

#include <math.h>

namespace mt_rrt::utils {
float to_rad(float angle) { return angle * PI / 180.f; }

float to_grad(float angle) { return angle * 180.f / PI; }

namespace {
float distance_(const float *a, const float *b, std::size_t size) {
  float res = 0;
  for (std::size_t k = 0; k < size; ++k) {
    res += powf(a[k] - b[k], 2.f);
  }
  res = sqrtf(res);
  return res;
}
} // namespace

float distance(const State &a, const State &b) {
  return distance_(a.data(), b.data(), a.size());
}

float distance(const Point &a, const Point &b) {
  return distance_(a.data(), b.data(), 2);
}

namespace {
float dot_(const float *a, const float *b, std::size_t size) {
  float res = 0;
  for (std::size_t k = 0; k < size; ++k) {
    res += a[k] * b[k];
  }
  return res;
}
} // namespace

float dot(const State &a, const State &b) {
  return dot_(a.data(), b.data(), a.size());
}

float dot(const Point &a, const Point &b) {
  return dot_(a.data(), b.data(), 2);
}

Point diff(const Point &a, const Point &b) {
  return {a[0] - b[0], a[1] - b[1]};
}

void add(Point& subject, const Point& to_add, const std::optional<float>& to_add_scale) {
    subject[0] += to_add_scale ? to_add[0] * to_add_scale.value() : to_add[0];
    subject[1] += to_add_scale ? to_add[1] * to_add_scale.value() : to_add[1];
}

void remove(Point& subject, const Point& to_remove, const std::optional<float>& to_remove_scale) {
    subject[0] -= to_remove_scale ? to_remove[0] * to_remove_scale.value() : to_remove[0];
    subject[1] -= to_remove_scale ? to_remove[1] * to_remove_scale.value() : to_remove[1];
}

Versor::Versor(float angle) {
  cos_sin[0] = cosf(angle);
  cos_sin[1] = sinf(angle);
}

Versor::Versor(const Point &vector_start, const Point &vector_end)
    : Versor(atan2f(vector_end[1] - vector_start[1],
                    vector_end[0] - vector_start[0])) {}

float curve_length(const std::vector<mt_rrt::State> &sequence) {
  float result = 0;
  for (std::size_t k = 1; k < sequence.size(); ++k) {
    result += distance(sequence[k - 1], sequence[k]);
  }
  return result;
}

namespace {
class Interpolator {
public:
  Interpolator(const std::vector<mt_rrt::State> &subject)
      : total_length(curve_length(subject)) {
    for (std::size_t k = 1; k < subject.size(); ++k) {
      const auto &start = subject[k - 1];
      const auto &end = subject[k];
      segments.emplace_back(Segment{start, end, distance(start, end)});
    }
  }

  State evaluate(const float s) const {
    float length = s * total_length;
    float cumulated_length = 0;
    for (const auto &segment : segments) {
      if (length <= (cumulated_length + segment.length)) {
        const float b_coeff = (length - cumulated_length) / segment.length;
        const float a_coeff = 1.f - b_coeff;
        State result;
        for (std::size_t k = 0; k < segment.end.size(); ++k) {
          result.push_back(a_coeff * segment.start[k] +
                           b_coeff * segment.end[k]);
        }
        return result;
      }
      cumulated_length += segment.length;
    }
    return segments.back().end;
  }

private:
  const float total_length;

  struct Segment {
    State start;
    State end;
    float length;
  };
  std::vector<Segment> segments;
};
} // namespace

float curve_similarity(const std::vector<mt_rrt::State> &a,
                       const std::vector<mt_rrt::State> &b) {
  float result = 0;
  const float delta = 1.f / static_cast<float>(100);
  Interpolator interp_a(a);
  Interpolator interp_b(b);
  std::size_t counter = 0;
  for (float s = delta; s <= 1.f; s += delta, ++counter) {
    result += distance(interp_a.evaluate(s), interp_b.evaluate(s));
  }
  return result / static_cast<float>(counter);
}

Segment::Segment(const Point &start, const Point &end) : start(start) {
  direction = diff(end, start);
  end_start_diff = &std::get<Point>(direction);
}

Segment::Segment(const Point &start, const Versor &direction) : start(start) {
  this->direction = &direction;
  this->end_start_diff = &direction.asPoint();
}

float closest_on_line(const Point &point, const Segment &segment) {
  const auto &b_a = segment.getEndStartDiff();
  Point c_a = diff(point, segment.getStart());
  return dot(c_a, b_a) / dot(b_a, b_a);
}

std::optional<std::array<float, 2>> closest_on_lines(const Segment &segment_a,
                                                     const Segment &segment_b) {
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

Point point_on_segment(float coeff, const Segment &segment) {
  Point result = segment.getStart();
  const auto &delta = segment.getEndStartDiff();
  result[0] += coeff * delta[0];
  result[1] += coeff * delta[1];
  return result;
}
} // namespace mt_rrt::utils
