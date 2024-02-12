/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <NavigationProblem.h>

#include <algorithm>
#include <optional>

namespace mt_rrt::navigation {
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
geom::Point relative_position(const geom::Sphere &obstacle,
                              const View &cart_state) {
  const float delta_x = obstacle.center.data()[0] - cart_state.data[0];
  const float delta_y = obstacle.center.data()[1] - cart_state.data[1];
  const float angle_cos = cosf(cart_state.data[2]);
  const float angle_sin = sinf(cart_state.data[2]);
  return {angle_cos * delta_x + angle_sin * delta_y,
          -angle_sin * delta_x + angle_cos * delta_y};
}

bool contains(float interval_min, float interval_max, float subject) {
  return (subject >= interval_min) && (subject <= interval_max);
}

float distance_point_segment(const geom::Point &point,
                             const geom::Point &segment_a,
                             const geom::Point &segment_b) {
  geom::Segment seg{segment_a, segment_b};
  float s = seg.closest_on_line(point);

  auto distance_eval = [&](float s) {
    auto point_on_segment = seg.at(s);
    return geom::distance(point, point_on_segment);
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

bool Cart::isCollisionPresent(const geom::Sphere &obstacle,
                              const View &cart_state) const {
  const auto relative_pos = relative_position(obstacle, cart_state);
  const auto &rel_pos_x = relative_pos.data()[0];
  const auto &rel_pos_y = relative_pos.data()[1];
  if (contains(cart_perimeter[1].data()[0], cart_perimeter[0].data()[0],
               rel_pos_x) &&
      contains(cart_perimeter[2].data()[1], cart_perimeter[0].data()[1],
               rel_pos_y)) {
    return true;
  }
  // get distance from sphere center and cart perimeter
  std::pair<const geom::Point *, const geom::Point *> segment_horizontal,
      segment_vertical;

  if (rel_pos_y > 0) {
    segment_horizontal =
        std::make_pair<const geom::Point *, const geom::Point *>(
            &cart_perimeter[0], &cart_perimeter[1]);
  } else {
    segment_horizontal =
        std::make_pair<const geom::Point *, const geom::Point *>(
            &cart_perimeter[2], &cart_perimeter[3]);
  }

  if (rel_pos_x > 0) {
    segment_vertical = std::make_pair<const geom::Point *, const geom::Point *>(
        &cart_perimeter[0], &cart_perimeter[3]);
  } else {
    segment_vertical = std::make_pair<const geom::Point *, const geom::Point *>(
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
compute_cart_trajectory_info(const View &start, const View &end,
                             const CartSteerLimits &steer_limits) {
  geom::Versor start_dir(start.data[2]), end_dir(end.data[2]);
  auto intersection_coefficients =
      geom::Segment::closest_on_lines(geom::Segment{start.trim(2), start_dir},
                                      geom::Segment{end.trim(2), end_dir});

  if (!intersection_coefficients.has_value()) {
    // start and end are aligned
    geom::Versor start_to_end(start.trim(2), end.trim(2));
    float dot_val = geom::dot(end_dir.asPoint(), start_to_end.asPoint());
    if (std::abs(dot_val - 1.f) < 1e-2f) {
      return TrivialLine{start, end};
    }
    return std::nullopt;
  }

  const auto [s, t] = intersection_coefficients.value();
  if ((s < 0) || (t > 0)) {
    return std::nullopt;
  }

  // direction of the bisec line, passing for the intersection and the center of
  // the blending arc
  geom::Versor gamma(
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

  geom::Point intersection_corner{start.trim(2)};
  intersection_corner = geom::sum(intersection_corner, start_dir.asPoint(), s);

  geom::Point center;
  {
    float intersection_center_distance = sqrtf(
        ray_info.distance * ray_info.distance + ray_info.ray * ray_info.ray);

    center = intersection_corner;
    center = geom::sum(center, gamma.asPoint(), intersection_center_distance);
  }
  geom::Point arc_begin = intersection_corner;
  arc_begin = geom::diff(arc_begin, start_dir.asPoint(), ray_info.distance);
  geom::Point arc_end = intersection_corner;
  arc_end = geom::sum(arc_end, end_dir.asPoint(), ray_info.distance);
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
    geom::Versor center_begin_arc(info.center, info.arc_begin);
    geom::Versor center_end_arc(info.center, info.arc_end);
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
  View getState() const final { return stateOnArc(angle_current); }
  float getCumulatedCost() const final {
    return ray * std::abs(angle_delta) * advancement_done;
  }

private:
  View stateOnArc(const float angle) const {
    currentStateCache_[0] = center.data()[0] + cosf(angle) * ray;
    currentStateCache_[1] = center.data()[1] + sinf(angle) * ray;
    currentStateCache_[2] =
        positive_negative_dir ? angle + geom::PI_HALF : angle - geom::PI_HALF;
    return View{currentStateCache_.data(), 3};
  }

  const TunneledConnector &caller;
  const geom::Point center;
  const float ray;
  bool positive_negative_dir;

  // !!!!! here angle refers to angle used to extapolate the cart baricenter
  // position it is NOT the cart orientation
  float angle_delta;
  float angle_current;
  std::size_t advancement_done = 0;
  std::size_t advancement_max;

  mutable std::array<float, 3> currentStateCache_;
};

class CartState {
protected:
  CartState(const geom::Point &point, float angle) {
    storage[0] = point.data()[0];
    storage[1] = point.data()[1];
    storage[2] = angle;
  }

  View getStorage() { return View{storage.data(), 3}; }

private:
  std::array<float, 3> storage;
};
} // namespace

class CartPosesConnector::CartLine : public CartState,
                                     public TunneledConnector::Line {
public:
  CartLine(const View &from, const geom::Point &to, float to_angle,
           const TunneledConnector &caller)
      : CartState{to, to_angle}, Line{from, getStorage(), caller} {}

  CartLine(const geom::Point &from, float from_angle, const View &to,
           const TunneledConnector &caller)
      : CartState{from, from_angle}, Line{getStorage(), to, caller} {}
};

// composite of sub-trajectories
class CartPosesConnector::CartTrajectory : public Trajectory {
public:
  CartTrajectory(const TunneledConnector &caller, CartTrajectoryInfo &&pieces)
      : info{std::forward<CartTrajectoryInfo>(pieces)} {
    struct Visitor {
      const TunneledConnector &caller;
      mutable std::vector<TrajectoryPtr> trajectories;

      void operator()(const Blended &arc) const {
        trajectories.emplace_back(std::make_unique<CartLine>(
            arc.start, arc.arc_begin, arc.start.data[2], caller));
        trajectories.emplace_back(std::make_unique<Arc>(arc, caller));
        trajectories.emplace_back(std::make_unique<CartLine>(
            arc.arc_end, arc.end.data[2], arc.end, caller));
      }

      void operator()(const TrivialLine &line) const {
        trajectories.emplace_back(
            std::make_unique<Line>(line.start, line.end, caller));
      }
    } visitor{caller};
    std::visit(visitor, info);
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
  View getState() const override {
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
  CartTrajectoryInfo info;
  std::size_t trajectories_it = 0;
  std::vector<TrajectoryPtr> trajectories;
};

CartPosesConnector::CartPosesConnector(const Scene &scene)
    : TunneledConnector(3) {
  this->scene = std::make_shared<Scene>(scene);
  setSteerDegree(STEER_DEGREE);
}

bool CartPosesConnector::checkAdvancement(const View &,
                                          const View &advanced_state) const {
  return std::any_of(
      scene->obstacles.begin(), scene->obstacles.end(),
      [cart = &scene->cart, &advanced_state](const geom::Sphere &obstacle) {
        return cart->isCollisionPresent(obstacle, advanced_state);
      });
}

TrajectoryPtr CartPosesConnector::getTrajectory(const View &start,
                                                const View &end) const {
  auto pieces =
      compute_cart_trajectory_info(start, end, scene->cart.steerLimits());
  if (pieces.has_value()) {
    return std::make_unique<CartTrajectory>(*this, std::move(pieces.value()));
  }
  return nullptr;
}

float CartPosesConnector::minCost2Go(const View &start, const View &end) const {
  const auto pieces =
      compute_cart_trajectory_info(start, end, scene->cart.steerLimits());
  if (pieces) {
    struct Visitor {
      const View &start;
      const View &end;
      mutable float result = 0;

      void operator()(const Blended &arc) const {
        geom::Versor center_begin(arc.center, arc.arc_begin);
        geom::Versor center_end(arc.center, arc.arc_end);
        float amplitude = center_begin.angleBetween(center_end);
        result += arc.ray * amplitude;
        result += geom::distance(View{start.data, 2}, arc.arc_begin.asView());
        result += geom::distance(View{end.data, 2}, arc.arc_end.asView());
      }

      void operator()(const TrivialLine &arc) const {
        result = geom::distance(start, end);
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
CartPosesConnector::make(const std::optional<Seed> &seed, const Scene &scene) {
  std::unique_ptr<CartPosesConnector> connector =
      std::make_unique<CartPosesConnector>(scene);

  std::vector<float> min_corner = {-5.f, -5.f, -geom::PI};
  std::vector<float> max_corner = {5.f, 5.f, geom::PI};

  std::shared_ptr<ProblemDescription> result;
  result.reset(new ProblemDescription{
      false, Positive<float>{GAMMA},
      std::make_unique<HyperBox>(min_corner, max_corner, seed),
      std::move(connector)});
  return result;
}
} // namespace mt_rrt::navigation