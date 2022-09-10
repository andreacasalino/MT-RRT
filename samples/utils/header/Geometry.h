/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/Node.h>

#include <array>
#include <math.h>
#include <optional>
#include <variant>

namespace mt_rrt::utils {
static constexpr float PI = 3.1415926535f;
static constexpr float PI_HALF = 0.5f * PI;

float to_rad(float angle);

float to_grad(float angle);

using Point = std::array<float, 2>;

float distance(const State &a, const State &b);
float distance(const Point &a, const Point &b);

float dot(const State &a, const State &b);
float dot(const Point &a, const Point &b);

// returns a - b
Point diff(const Point &a, const Point &b);

void add(Point &subject, const Point &to_add,
         const std::optional<float> &to_add_scale = std::nullopt);
void remove(Point &subject, const Point &to_remove,
            const std::optional<float> &to_remove_scale = std::nullopt);

class Versor {
public:
  Versor(float angle);

  Versor(const Point &vector_start, const Point &vector_end);

  const Point &asPoint() const { return cos_sin; };

  float cos() const { return cos_sin[0]; }
  float sin() const { return cos_sin[1]; }

  float angle() const { return atan2f(cos_sin[1], cos_sin[0]); }

  float cross(const Versor &o) const {
    return this->cos_sin[0] * o.cos_sin[1] - this->cos_sin[1] * o.cos_sin[0];
  }

  float angleBetween(const Versor &o) const {
    float cos_val = utils::dot(this->asPoint(), o.asPoint());
    return acosf(cos_val);
  }

private:
  Point cos_sin;
};

struct Sphere {
  Positive<float> ray;
  Point center;
};

float curve_length(const std::vector<mt_rrt::State> &sequence);

float curve_similarity(const std::vector<State> &a,
                       const std::vector<State> &b);

class Segment {
public:
  Segment(const Point &start, const Point &end);
  Segment(const Point &start, const Versor &direction);

  const Point &getStart() const { return start; }
  const Point &getEndStartDiff() const { return *end_start_diff; }

private:
  const Point &start;

  const Point *end_start_diff;
  std::variant<const Versor *, Point> direction;
};

float closest_on_line(const Point &point, const Segment &segment);

std::optional<std::array<float, 2>> closest_on_lines(const Segment &segment_a,
                                                     const Segment &segment_b);

Point point_on_segment(float coeff, const Segment &segment);
} // namespace mt_rrt::utils
