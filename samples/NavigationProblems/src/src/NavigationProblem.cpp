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
CartSteerLimits::CartSteerLimits() : CartSteerLimits(0.1f, 1.f) {}

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

Point to_point(const State &state) { return Point{state[0], state[1]}; }

namespace {
State to_state(const Point &point, float angle) {
  return State{point[0], point[1], angle};
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

  if (!intersection_coefficients.has_value()) {
    // start and end are aligned
    utils::Versor start_to_end(to_point(start), to_point(end));
    float dot_val = utils::dot(end_dir.asPoint(), start_to_end.asPoint());
    if (std::abs(dot_val - 1.f) < 1e-2f) {
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
  float theta = gamma.angleBetween(end_dir);
  auto ray_info = from_distance(theta, std::min(s, -t));
  if (ray_info.ray < steer_limits.minRadius()) {
    return std::nullopt;
  }
  if (ray_info.ray > steer_limits.maxRadius()) {
    ray_info = from_ray(theta, steer_limits.maxRadius());
  }

  Point intersection_corner = {start[0], start[1]};
  utils::add(intersection_corner, start_dir.asPoint(), s);

  Point center;
  {
    float intersection_center_distance = sqrtf(
        ray_info.distance * ray_info.distance + ray_info.ray * ray_info.ray);

    center = intersection_corner;
    utils::add(center, gamma.asPoint(), intersection_center_distance);
  }
  Point arc_begin = intersection_corner;
  utils::remove(arc_begin, start_dir.asPoint(), ray_info.distance);
  Point arc_end = intersection_corner;
  utils::add(arc_end, end_dir.asPoint(), ray_info.distance);
  return Blended{start,
                 end,
                 ray_info.ray,
                 std::move(center),
                 std::move(arc_begin),
                 std::move(arc_end)};
}

namespace {
class Arc : public Trajectory {
public:
  Arc(const Blended &info, const TunneledConnector &caller)
      : caller(caller), center(info.center), ray(info.ray) {
    utils::Versor center_begin_arc(info.center, info.arc_begin);
    utils::Versor center_end_arc(info.center, info.arc_end);
    const float amplitude = center_begin_arc.angleBetween(center_end_arc);
    advancement_max = static_cast<std::size_t>(
        std::ceil(ray * amplitude / caller.getSteerDegree()));
    angle_current = center_begin_arc.angle();
    positive_negative_dir = 0.f < center_begin_arc.cross(center_end_arc);
    angle_delta = amplitude / static_cast<float>(advancement_max);
    if (!positive_negative_dir) {
      angle_delta = -angle_delta;
    }
  }

  AdvanceInfo advance() final {
    ++advancement_done;
    angle_current += angle_delta;
    const auto advanced_state = stateOnArc(angle_current);
    if (advancement_done == advancement_max) {
      if (caller.checkAdvancement(advanced_state, advanced_state)) {
        return AdvanceInfo::blocked;
      }
      return AdvanceInfo::targetReached;
    }
    auto const result = caller.checkAdvancement(advanced_state, advanced_state)
                            ? AdvanceInfo::blocked
                            : AdvanceInfo::advanced;
    return result;
  }
  State getState() const final { return stateOnArc(angle_current); }
  float getCumulatedCost() const final {
    return ray * std::abs(angle_delta) * advancement_done;
  }

private:
  State stateOnArc(const float angle) const {
    float position_x, position_y;
    position_x = center[0] + cosf(angle) * ray;
    position_y = center[1] + sinf(angle) * ray;
    return State{position_x, position_y,
                 positive_negative_dir ? angle + utils::PI_HALF
                                       : angle - utils::PI_HALF};
  }

  const TunneledConnector &caller;
  const Point center;
  const float ray;
  bool positive_negative_dir;

  // !!!!! here angle refers to angle used to extapolate the cart baricenter
  // position it is NOT the cart orientation
  float angle_delta;
  float angle_current;
  std::size_t advancement_done = 0;
  std::size_t advancement_max;
};
} // namespace

// composite of sub-trajectories
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
      if (trajectories_it == (trajectories.size() - 1)) {
        return info;
      }
      ++trajectories_it;
      return AdvanceInfo::advanced;
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
        float amplitude = center_begin.angleBetween(center_end);
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

const float CartPosesConnector::STEER_DEGREE = 0.2f;

namespace {
static constexpr float GAMMA = 5.f;
}

std::shared_ptr<ProblemDescription>
make_problem_description(const std::optional<Seed> &seed, const Scene &scene) {
  std::unique_ptr<CartPosesConnector> connector =
      std::make_unique<CartPosesConnector>(scene);

  State min_corner = {-5.f, -5.f, -utils::PI};
  State max_corner = {5.f, 5.f, utils::PI};

  std::shared_ptr<ProblemDescription> result;
  result.reset(new ProblemDescription{
      std::make_unique<HyperBox>(min_corner, max_corner, seed),
      std::move(connector), true, Positive<float>{GAMMA}});
  return result;
}
} // namespace mt_rrt::samples