/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <Geometry.h>
#include <Transform.h>

namespace mt_rrt::geom {
class Versor {
public:
  Versor(float angle);

  Versor(const Point &vector_start, const Point &vector_end);

  const Point &asPoint() const { return cos_sin; };

  Point asPointScaled(float scale) const {
    return Point{cos_sin.data()[0] * scale, cos_sin.data()[1] * scale};
  }

  float cos() const { return cos_sin.data()[0]; }
  float sin() const { return cos_sin.data()[1]; }

  float angle() const { return atan2f(cos_sin.data()[1], cos_sin.data()[0]); }

  float cross(const Versor &o) const {
    return this->cos_sin.data()[0] * o.cos_sin.data()[1] -
           this->cos_sin.data()[1] * o.cos_sin.data()[0];
  }

  float angleBetween(const Versor &o) const {
    float cos_val = dot(this->asPoint(), o.asPoint());
    return acosf(cos_val);
  }

private:
  Point cos_sin;
};

struct Sphere {
  Positive<float> ray;
  Point center;
};

class Segment {
public:
  Segment(const Point &start, const Point &end);
  Segment(const Point &start, const Versor &direction);

  const Point &getStart() const { return start; }
  const Point &getEnd() const { return end; }
  const Point &getEndStartDiff() const { return end_start_diff; }

  float closest_on_line(const Point &point) const;

  static std::optional<std::array<float, 2>>
  closest_on_lines(const Segment &segment_a, const Segment &segment_b);

  Point at(float coeff) const;

private:
  Point start;
  Point end;
  Point end_start_diff;
};

struct Box {
  Box(const Point &min, const Point &max,
      const std::optional<Transform> &trsf = std::nullopt);

  Point min_corner; // seen from local frame!!!
  Point max_corner; // seen from local frame!!!

  std::optional<Transform> trsf;

  bool collides(const Segment &segment) const;

  bool collides(const Point &segment_start, const Point &segment_end) const {
    return collides(Segment{segment_start, segment_end});
  }
};
using Boxes = std::vector<Box>;

} // namespace mt_rrt::geom
