/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <NavigationProblem.h>

namespace mt_rrt::samples {
float to_rad(float angle) { return angle * PI / 180.f; }

float to_grad(float angle) { return angle * 180.f / PI; }

Cart::Cart(float width, float length) {
  this->width.set(width);
  this->length.set(length);
  cart_perimeter[0] = {width, length};
  cart_perimeter[1] = {-width, length};
  cart_perimeter[2] = {-width, -length};
  cart_perimeter[3] = {width, -length};
}

namespace {
struct Pos_ {
  float x;
  float y;
};
Pos_ relative_position(const Sphere &obstacle, const State &cart_state) {
  const float delta_x = obstacle.center[0] - cart_state[0];
  const float delta_y = obstacle.center[1] - cart_state[1];
  const float angle_cos = cosf(cart_state[2]);
  const float angle_sin = sinf(cart_state[2]);
  return {angle_cos * delta_x + angle_sin * delta_y,
          -angle_sin * delta_x + angle_cos * delta_y};
}

bool contains(float interval_min, float interval_max, float subject) {
  return (subject >= interval_min) && (subject <= interval_max);
}

// a - b
Point diff(const Point &a, const Point &b) {
  return {a[0] - b[0], a[1] - b[1]};
}

float dot(const Point &a, const Point &b) { return a[0] * b[0] + a[1] * b[1]; }

float distance_point_segment(const Point &point, const Point &segment_a,
                             const Point &segment_b) {
  Point b_a = diff(segment_b, segment_a);
  Point c_a = diff(point, segment_a);
  auto distance_eval = [&](float s) {
    Point point_on_segment = point;
    point_on_segment[0] += s * b_a[0];
    point_on_segment[1] += s * b_a[1];
    float result = powf(point[0] - point_on_segment[0], 2.f);
    result += powf(point[1] - point_on_segment[1], 2.f);
    return sqrtf(result);
  };

  float s = dot(c_a, b_a) / dot(b_a, b_a);
  if (s < 0) {
    return distance_eval(0);
  }
  if (s > 1.f) {
    return distance_eval(1.f);
  }
  return distance_eval(s);
}
} // namespace

bool Cart::isCollisionPresent(const Cart &cart, const Sphere &obstacle,
                              const State &cart_state) const {
  auto &&[rel_pos_x, rel_pos_y] = relative_position(obstacle, cart_state);
  if (contains(-cart.width.get(), cart.width.get(), rel_pos_x) &&
      contains(-cart.length.get(), cart.length.get(), rel_pos_y)) {
    return true;
  }
  // get distance from sphere center and cart perimeter
  float distance = distance_point_segment(obstacle.center, cart_perimeter[0],
                                          cart_perimeter[1]);
  distance = std::min(distance,
                      distance_point_segment(obstacle.center, cart_perimeter[1],
                                             cart_perimeter[2]));
  distance = std::min(distance,
                      distance_point_segment(obstacle.center, cart_perimeter[2],
                                             cart_perimeter[3]));
  distance = std::min(distance,
                      distance_point_segment(obstacle.center, cart_perimeter[3],
                                             cart_perimeter[0]));

  return distance <= obstacle.ray.get();
}
} // namespace mt_rrt::samples