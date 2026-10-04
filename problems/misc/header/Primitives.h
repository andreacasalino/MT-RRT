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

  PointAllocated asPointScaled(float scale) const {
    return PointAllocated{cos_sin.x() * scale, cos_sin.y() * scale};
  }

  float cos() const { return cos_sin.x(); }
  float sin() const { return cos_sin.y(); }

  [[discard]] float angle() const { return atan2f(cos_sin.y(), cos_sin.x()); }

  [[discard]] float cross(const Versor &o) const {
    return this->cos_sin.x() * o.cos_sin.y() -
           this->cos_sin.y() * o.cos_sin.x();
  }

  [[discard]] float angleBetween(const Versor &o) const {
    float cos_val = dot_product(this->asPoint(), o.asPoint());
    return acosf(cos_val);
  }

private:
  PointAllocated cos_sin;
};

struct Sphere {
  Positive ray;
  Point center;
};

class Segment {
public:
  Segment(const Point &start, const Point &end);
  Segment(const Point &start, const Versor &direction);

  const Point &getStart() const { return start; }
  const Point &getEnd() const { return end; }
  const Point &getEndStartDiff() const { return end_start_diff; }

  [[discard]] float closest_on_line(const Point &point) const;

  [[discard]] static std::optional<std::array<float, 2>>
  closest_between_lines(const Segment &segment_a, const Segment &segment_b);

  [[discard]] Point at(float coeff) const;

private:
  PointAllocated start;
  PointAllocated end;
  PointAllocated end_start_diff;
};

struct Box {
  Box(const Point &min, const Point &max);
  Box(const Point &min, const Point &max, const Transform &t) : Box{min, max} {
    trsf.emplace(t);
  }

  PointAllocated min_corner; // seen from local frame!!!
  PointAllocated max_corner; // seen from local frame!!!

  std::optional<Transform> trsf;

  [[discard]] bool collides(const Segment &segment) const;

  [[discard]] bool collides(const Point &segment_start,
                            const Point &segment_end) const {
    return collides(Segment{segment_start, segment_end});
  }
};
using Boxes = std::vector<Box>;

} // namespace mt_rrt::geom
