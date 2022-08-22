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
CartSteerLimits::CartSteerLimits() : CartSteerLimits(0, 1.f) {}

CartSteerLimits::CartSteerLimits(float min_radius, float max_radius)
    : min_steer_radius(min_radius), max_steer_radius(max_radius) {
  if (max_radius <= min_radius) {
    throw Error{"Invalid cart steer limits"};
  }
}

Cart::Cart(float width, float length, const CartSteerLimits &steer_limits)
    : steer_limits(steer_limits) {
  this->width.set(width);
  this->length.set(length);
  cart_perimeter[0] = {0.5f * length, 0.5f * width};
  cart_perimeter[1] = {-0.5f * length, 0.5f * width};
  cart_perimeter[2] = {-0.5f * length, -0.5f * width};
  cart_perimeter[3] = {0.5f * length, -0.5f * width};
}

namespace {
Point to_point(const State &state) { return Point{state[0], state[1]}; }

State to_state(const Point &point, float angle) {
  return State{point[0], point[1], angle};
}

void add(float *subject, float delta_x, float delta_y) {
  subject[0] += delta_x;
  subject[1] += delta_y;
}

Point relative_position(const Sphere &obstacle, const State &cart_state) {
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

float distance_point_segment(const Point &point, const Point &segment_a,
                             const Point &segment_b) {
  utils::Segment seg{segment_a, segment_b};
  float s = utils::closest_on_line(point, seg);

  auto distance_eval = [&](float s) {
    auto point_on_segment = utils::point_on_segment(s, seg);
    return utils::distance(point, point_on_segment);
  };

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
  const auto relative_pos = relative_position(obstacle, cart_state);
  const auto &rel_pos_x = relative_pos[0];
  const auto &rel_pos_y = relative_pos[1];
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
      relative_pos, *segment_horizontal.first, *segment_horizontal.second);
  distance = std::min(
      distance, distance_point_segment(relative_pos, *segment_vertical.first,
                                       *segment_vertical.second));

  return distance <= obstacle.ray.get();
}

namespace {
float angle_between(const utils::Versor &a, const utils::Versor &b) {
  float cos_val = utils::dot(a.asPoint(), b.asPoint());
  return acosf(cos_val);
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
} // namespace

std::optional<CartTrajectoryInfo>
compute_cart_trajectory_info(const State &start, const State &end,
                             const CartSteerLimits &steer_limits) {
  utils::Versor start_dir(start[2]), end_dir(end[2]);
  auto intersection_coefficients =
      utils::closest_on_lines(utils::Segment{to_point(start), start_dir},
                              utils::Segment{to_point(end), end_dir});

  if (intersection_coefficients == std::nullopt) {
    // start and end are aligned
    utils::Versor start_to_end(to_point(start), to_point(end));
    float dot_val = utils::dot(start_dir.asPoint(), start_to_end.asPoint());
    if (std::abs(dot_val - 1.f) < 1e-3) {
      return TrivialLine{start, end};
    }
    return std::nullopt;
  }

  const auto &[s, t] = intersection_coefficients.value();
  if ((s < 0) || (t > 0)) {
    return std::nullopt;
  }

  // direction of the bisec line, passing for the intersection and the center of
  // the blending arc
  utils::Versor gamma(
      atan2f(end_dir.sin() - start_dir.sin(), end_dir.cos() - start_dir.cos()));

  // angular aplitude between bisec line and one of the line where start or end
  // lies
  float theta = angle_between(gamma, end_dir);
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
    float intersection_center_distance = sqrtf(
        ray_info.distance * ray_info.distance + ray_info.ray * ray_info.ray);

    center = intersection_corner;
    add(center.data(), intersection_center_distance * gamma.cos(),
        intersection_center_distance * gamma.sin());
  }
  Point arc_begin = intersection_corner;
  arc_begin[0] -= ray_info.distance * start_dir.cos();
  arc_begin[1] -= ray_info.distance * start_dir.sin();
  Point arc_end = intersection_corner;
  arc_end[0] += ray_info.distance * end_dir.cos();
  arc_end[1] += ray_info.distance * end_dir.sin();
  return Blended{start,
                 end,
                 ray_info.ray,
                 std::move(center),
                 std::move(arc_begin),
                 std::move(arc_end)};
}

namespace {
class ArcAngularRange {
public:
  ArcAngularRange(const Blended &info, float steer_degree) {
    utils::Versor center_begin(info.center, info.arc_begin);
    utils::Versor center_end(info.center, info.arc_end);
    float amplitude = angle_between(center_begin, center_end);
    arc_angle_current = center_begin.angle();
    float steer_degree_angular = steer_degree / info.ray;
    advancements_required =
        static_cast<std::size_t>(std::ceil(amplitude / steer_degree_angular));
    arc_angle_delta = amplitude / static_cast<float>(advancements_required);
    rotation_direction = center_begin.cross(center_end) > 0;
  }

  struct Angles {
    float arc_angle;
    float cart_orientation;
    bool is_last;
  };
  Angles next() const {
    Angles result;
    result.arc_angle = arc_angle_current;
    result.arc_angle += (rotation_direction ? 1.f : -1.f) * arc_angle_delta;
    result.cart_orientation = result.arc_angle;
    result.cart_orientation +=
        (rotation_direction ? 1.f : -1.f) * utils::PI_HALF;
    result.is_last = (advanced + 1) == advancements_required;
    return result;
  }

  void advance() {
    arc_angle_current += (rotation_direction ? 1.f : -1.f) * arc_angle_delta;
    ++advanced;
  }

  float amplitudeSoFar() const { return arc_angle_delta * advanced; }

private:
  bool rotation_direction; // true = positive, false = negative

  std::size_t advancements_required;
  std::size_t advanced = 0;

  float arc_angle_current;
  float arc_angle_delta;
};

class Arc : public Trajectory {
public:
  Arc(const Blended &info, const TunneledConnector &caller)
      : caller(caller), info(info),
        angular_range(info, caller.getSteerDegree()) {
    state_current = info.start;
  }

  AdvanceInfo advance() final {
    const auto info_next = angular_range.next();
    State advanced = stateOnArc(info_next);
    if (info_next.is_last) {
      if (caller.checkAdvancement(advanced, advanced)) {
        return AdvanceInfo::blocked;
      }
      state_current = std::move(advanced);
      angular_range.advance();
      return AdvanceInfo::targetReached;
    }
    auto const result = caller.checkAdvancement(advanced, advanced)
                            ? AdvanceInfo::blocked
                            : AdvanceInfo::advanced;
    if (result == AdvanceInfo::advanced) {
      state_current = std::move(advanced);
      angular_range.advance();
    }
    return result;
  }
  State getState() const final { return state_current; }
  float getCumulatedCost() const final {
    return info.ray * angular_range.amplitudeSoFar();
  }

private:
  State stateOnArc(const ArcAngularRange::Angles &info_angles) const {
    float position_x, position_y;
    position_x = info.center[0] + cosf(info_angles.arc_angle) * info.ray;
    position_y = info.center[1] + sinf(info_angles.arc_angle) * info.ray;
    return State{position_x, position_y, info_angles.cart_orientation};
  }

  const TunneledConnector &caller;
  const Blended info;

  ArcAngularRange angular_range;
  State state_current;
};
} // namespace

class CartPosesConnector::CartTrajectory : public Trajectory {
public:
  CartTrajectory(const TunneledConnector &caller,
                 const CartTrajectoryInfo &pieces) {
    struct Visitor {
      const TunneledConnector &caller;
      const CartTrajectoryInfo &pieces;
      mutable std::vector<TrajectoryPtr> trajectories;

      void operator()(const Blended &arc) const {
        trajectories.emplace_back(std::make_unique<Line>(
            arc.start, to_state(arc.arc_begin, arc.start[2]), caller));
        trajectories.emplace_back(std::make_unique<Arc>(arc, caller));
        trajectories.emplace_back(std::make_unique<Line>(
            to_state(arc.arc_end, arc.end[2]), arc.end, caller));
      }

      void operator()(const TrivialLine &line) const {
        trajectories.emplace_back(
            std::make_unique<Line>(line.start, line.end, caller));
      }
    } visitor{caller, pieces};
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
      result += trajectories[index]->getCumulatedCost();
    }
    return result;
  }

private:
  std::vector<TrajectoryPtr> trajectories;
  std::size_t trajectories_it = 0;
};

CartPosesConnector::CartPosesConnector(const Scene &scene)
    : TunneledConnector(3) {
  this->scene = std::make_shared<Scene>(scene);
  setSteerDegree(STEER_DEGREE);
}

bool CartPosesConnector::checkAdvancement(const State &,
                                          const State &advanced_state) const {
  return std::any_of(
      scene->obstacles.begin(), scene->obstacles.end(),
      [cart = &scene->cart, &advanced_state](const Sphere &obstacle) {
        return cart->isCollisionPresent(obstacle, advanced_state);
      });
}

TrajectoryPtr CartPosesConnector::getTrajectory(const State &start,
                                                const State &end) const {
  const auto pieces =
      compute_cart_trajectory_info(start, end, scene->cart.steerLimits());
  if (pieces) {
    return std::make_unique<CartTrajectory>(*this, pieces.value());
  }
  return nullptr;
}

float CartPosesConnector::minCost2Go(const State &start,
                                     const State &end) const {
  const auto pieces =
      compute_cart_trajectory_info(start, end, scene->cart.steerLimits());
  if (pieces) {
    struct Visitor {
      const State &start;
      const State &end;
      mutable float result = 0;

      void operator()(const Blended &arc) const {
        utils::Versor center_begin(arc.center, arc.arc_begin);
        utils::Versor center_end(arc.center, arc.arc_end);
        float amplitude = angle_between(center_begin, center_end);
        result += arc.ray * amplitude;
        result += utils::distance(to_point(start), arc.arc_begin);
        result += utils::distance(to_point(end), arc.arc_end);
      }

      void operator()(const TrivialLine &arc) const {
        result = utils::distance(start, end);
      }
    } visitor{start, end};
    std::visit(visitor, pieces.value());
    return visitor.result;
  }
  return COST_MAX;
}

const float CartPosesConnector::STEER_DEGREE = 0; // TODO
} // namespace mt_rrt::samples