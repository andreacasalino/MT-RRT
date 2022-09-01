/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <NavigationProblem.h>

#include <algorithm>
#include <optional>

namespace mt_rrt::samples {
float to_rad(float angle) { return angle * PI / 180.f; }

float to_grad(float angle) { return angle * 180.f / PI; }

CartSteerLimits::CartSteerLimits(float min_radius, float max_radius)
 : min_steer_radius(min_radius), max_steer_radius(max_radius) {
    if (max_radius >= min_radius) {
        throw Error{"Invalid cart steer limits"};
    }
}

Cart::Cart(float width, float length, const CartSteerLimits &steer_limits)
    : steer_limits(steer_limits) {
  this->width.set(width);
  this->length.set(length);
  cart_perimeter[0] = {0.5f * width, 0.5f * length};
  cart_perimeter[1] = {-0.5f * width, 0.5f * length};
  cart_perimeter[2] = {-0.5f * width, -0.5f * length};
  cart_perimeter[3] = {0.5f * width, -0.5f * length};
}

namespace {
// a - b
Point diff(const Point& a, const Point& b) {
    return { a[0] - b[0], a[1] - b[1] };
}

float dot(const float* a, const float* b) { return a[0] * b[0] + a[1] * b[1]; }

Point to_point(const State& state) { return Point{ state[0], state[1] }; }

void add(float* subject, float delta_x, float delta_y) {
    subject[0] += delta_x;
    subject[1] += delta_y;
}

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

float distance(const float *a, const float *b) {
  float result = powf(a[0] - b[0], 2.f);
  result += powf(a[1] - b[1], 2.f);
  return sqrtf(result);
}

float distance_point_segment(const Point &point, const Point &segment_a,
                             const Point &segment_b) {
  Point b_a = diff(segment_b, segment_a);
  Point c_a = diff(point, segment_a);
  auto distance_eval = [&](float s) {
    Point point_on_segment = point;
    point_on_segment[0] += s * b_a[0];
    point_on_segment[1] += s * b_a[1];
    return distance(point.data(), point_on_segment.data());
  };

  float s = dot(c_a.data(), b_a.data()) / dot(b_a.data(), b_a.data());
  if (s < 0) {
    return distance_eval(0);
  }
  if (s > 1.f) {
    return distance_eval(1.f);
  }
  return distance_eval(s);
}
} // namespace

bool Cart::isCollisionPresent(const Sphere &obstacle,
                              const State &cart_state) const {
  auto &&[rel_pos_x, rel_pos_y] = relative_position(obstacle, cart_state);
  if (contains(cart_perimeter[1][0], cart_perimeter[0][0], rel_pos_x) &&
      contains(cart_perimeter[2][1], cart_perimeter[0][1], rel_pos_y)) {
    return true;
  }
  // get distance from sphere center and cart perimeter
  std::pair<const Point *, const Point *> segment_horizontal, segment_vertical;
  if (rel_pos_y > 0) {
    segment_horizontal = std::make_pair<const Point *, const Point *>(
        &cart_perimeter[0], &cart_perimeter[1]);
  } else {
    segment_horizontal = std::make_pair<const Point *, const Point *>(
        &cart_perimeter[2], &cart_perimeter[3]);
  }
  if (rel_pos_x > 0) {
    segment_vertical = std::make_pair<const Point *, const Point *>(
        &cart_perimeter[0], &cart_perimeter[3]);
  } else {
    segment_vertical = std::make_pair<const Point *, const Point *>(
        &cart_perimeter[1], &cart_perimeter[2]);
  }

  float distance = distance_point_segment(
      obstacle.center, *segment_horizontal.first, *segment_horizontal.second);
  distance = std::min(
      distance, distance_point_segment(obstacle.center, *segment_vertical.first,
                                       *segment_vertical.second));

  return distance <= obstacle.ray.get();
}

namespace {
    class Versor {
    public:
        Versor(float angle) {
            cos_sin[0] = cosf(angle);
            cos_sin[1] = sinf(angle);
        }

        const Point& asPoint() const { return cos_sin; };

        float cos() const { return cos_sin[0]; }
        float sin() const { return cos_sin[1]; }

    private:
        Point cos_sin;
    };

    std::optional<std::array<float, 2>>
        compute_intersection_coefficients(const State& start, const State& end,
            const Versor& start_dir,
            const Versor& end_dir) {
        const Point& V1 = start_dir.asPoint();
        const Point& V2 = end_dir.asPoint();
        const Point V0 = { start[0] - end[0], start[1] - end[1] };
        const float m00 = dot(V1.data(), V1.data());
        const float m11 = dot(V2.data(), V2.data());
        const float m01 = -dot(V1.data(), V2.data());
        const float c0 = -dot(V0.data(), V1.data());
        const float c1 = dot(V0.data(), V2.data());
        const float determinant = m00 * m11 - m01 * m01;
        if (std::abs(determinant) < 0.0001f) {
            return std::nullopt;
        }
        const float s_min = (c0 * m11 - m01 * c1) / determinant;
        const float t_min = (c1 - m01 * s_min) / m11;
        return std::array<float, 2>{s_min, t_min};
    }

    struct RayAndDistanceToIntersection {
        float distance;
        float ray;
    };
    RayAndDistanceToIntersection from_distance(float theta, float dist) {
        RayAndDistanceToIntersection result;
        result.distance = dist;
        result.ray = tanf(theta) * dist;
        return result;
    }
    RayAndDistanceToIntersection from_ray(float theta, float ray) {
        RayAndDistanceToIntersection result;
        result.ray = ray;
        result.distance = ray / tanf(theta);
        return result;
    }

}

std::optional<TrajectoryInfo>
compute_cart_trajectory_info(const State& start, const State& end,
    const CartSteerLimits& steer_limits) {
  Versor start_dir(start[2]), end_dir(end[2]);
  auto intersection_coefficients = compute_intersection_coefficients(start, end, start_dir, end_dir);

  if (intersection_coefficients == std::nullopt) {
    // start and end are aligned
    if (dot(start.data(), end.data()) < 0) {
      return std::nullopt;
    }
    // check the 2 states lies on exactly the same line
    Point end2 = end_dir.asPoint();
    add(end2.data(), end.data()[0], end.data()[1]);
    if (distance_point_segment(to_point(start), to_point(end), end2) < 1e-4f) {
        return TrivialLine{};
    }
  }

  const auto &[s, t] = intersection_coefficients.value();
  if ((s < 0) || (t > 0)) {
    return std::nullopt;
  }

  // angle of the bisec line, passing for the intersection and the center of the blending arc
  float gamma =
      atan2f(end_dir.cos() - start_dir.cos(), end_dir.sin() - start_dir.sin());

  // angular aplitude between bisec line and one of the line where start or end lies
  float theta = end[2] - gamma;
  auto ray_info = from_distance(theta, std::min(s, -t));
  if (ray_info.ray < steer_limits.minRadius()) {
    return std::nullopt;
  }
  if (ray_info.ray > steer_limits.maxRadius()) {
    ray_info = from_ray(theta, steer_limits.maxRadius());
  }

  Point intersection_corner = {start[0], start[1]};
  add(intersection_corner.data(), start_dir.cos() * s, start_dir.sin() * s);

  Point center;
  {
    float intersection_center_distance =
        sqrtf(ray_info.distance * ray_info.distance + ray_info.ray * ray_info.ray);

    center = intersection_corner;
    add(center.data(), intersection_center_distance * cosf(gamma), intersection_center_distance * sinf(gamma));
  }
  Point arc_begin = intersection_corner;
  arc_begin[0] -= ray_info.distance * start_dir.cos();
  arc_begin[1] -= ray_info.distance * start_dir.sin();
  Point arc_end = intersection_corner;
  arc_end[0] += ray_info.distance * end_dir.cos();
  arc_end[1] += ray_info.distance * end_dir.sin();
  return BlendingArc{ray_info.ray, std::move(center), std::move(arc_begin), std::move(arc_end) };
}

namespace {
// absolute angle of the segment going from a to b
float compute_angle(const Point& a, const Point& b) {
    return atan2f(b[1] - a[1], b[0] - a[0]);
}

std::array<float, 2> arc_angles(const BlendingArc& arc) {
    return { compute_angle(arc.center, arc.arc_begin),
        compute_angle(arc.center, arc.arc_end) };
}

class BlendingTrajectory : public Trajectory {
public:
    BlendingTrajectory(const BlendingArc& info);
};

class TrajectoryComposite : public Trajectory {
public:
  TrajectoryComposite(const TunneledConnector& caller,
                      const TrajectoryInfo&pieces) {
      struct Visitor {
          const TunneledConnector& caller;
          const TrajectoryInfo& pieces;
          mutable std::vector<TrajectoryPtr> trajectories;

          void operator()(const BlendingArc& arc) const {
              // TODO put in trajectories
              // Line start -> arc_begin
              // BlendingTrajectory for arc
              // Line arc_end -> end
          }

          void operator()(const TrivialLine& arc) const {
              // TODO put in trajectories
              // Line start -> end
          }
      } visitor{ caller, pieces };
      std::visit(visitor, pieces);
      trajectories = std::move(visitor.trajectories);
  }

  AdvanceInfo advance() override {
    auto info = trajectories[trajectories_it]->advance();
    if (info == AdvanceInfo::targetReached) {
      ++trajectories_it;
      if (trajectories_it == trajectories.size()) {
        return AdvanceInfo::targetReached;
      } else {
        return AdvanceInfo::advanced;
      }
    }
    return info;
  }
  State getState() const override {
    return trajectories[trajectories_it]->getState();
  }
  float getCumulatedCost() const override {
    float result = 0;
    for (std::size_t index = 0; index <= trajectories_it; ++index) {
      result += trajectories[trajectories_it]->getCumulatedCost();
    }
    return result;
  }

private:
  std::vector<TrajectoryPtr> trajectories;
  std::size_t trajectories_it;
};
} // namespace

bool CartPosesConnector::checkAdvancement(const State& previous_state,
    const State& advanced_state) const {
    return std::any_of(scene->obstacles.begin(), scene->obstacles.end(), [cart = &scene->cart, &advanced_state](const Sphere& obstacle) {
        return cart->isCollisionPresent(obstacle, advanced_state);
        });
}

TrajectoryPtr CartPosesConnector::getTrajectory(const State &start,
                                                const State &end) const {
  const auto pieces = compute_cart_trajectory_info(start, end, scene->cart.steerLimits());
  if (pieces) {
    return std::make_unique<TrajectoryComposite>(static_cast<const TunneledConnector&>(*this), pieces.value());
  }
  return nullptr;
}

float CartPosesConnector::minCost2Go(const State &start,
                                     const State &end) const {
  const auto pieces = compute_cart_trajectory_info(start, end, scene->cart.steerLimits());
  if (pieces) {
      struct Visitor {
          const State& start;
          const State& end;
          mutable float result = 0;

          void operator()(const BlendingArc& arc) const {
              auto&& [arc_begin_angle, arc_end_angle] = arc_angles(arc);
              result += arc.ray * std::abs(arc_end_angle - arc_begin_angle);
              result += distance(start.data(), arc.arc_begin.data());
              result += distance(end.data(), arc.arc_end.data());
          }

          void operator()(const TrivialLine& arc) const {
              result = distance(start.data(), end.data());
          }
      } visitor{start, end};
      std::visit(visitor, pieces.value());
    return visitor.result;
  }
  return COST_MAX;
}
} // namespace mt_rrt::samples