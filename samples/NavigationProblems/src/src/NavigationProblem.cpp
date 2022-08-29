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

Cart::Cart(float width, float length) {
  this->width.set(width);
  this->length.set(length);
  cart_perimeter[0] = {0.5f * width, 0.5f * length};
  cart_perimeter[1] = {-0.5f * width, 0.5f * length};
  cart_perimeter[2] = {-0.5f * width, -0.5f * length};
  cart_perimeter[3] = {0.5f * width, -0.5f * length};
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

float distance(const float *a, const float *b) {
  float result = powf(a[0] - b[0], 2.f);
  result += powf(a[1] - b[1], 2.f);
  return sqrtf(result);
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
    return distance(point.data(), point_on_segment.data());
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
struct LinePiece {
  State start;
  State end;
};
struct ArcPiece {
  State center;
  float ray;
  float angle_start;
  float angle_end;
};
struct TrajectoryPieces {
  std::optional<LinePiece> line_start;
  std::optional<ArcPiece> arc;
  std::optional<LinePiece> line_end;
};
std::optional<TrajectoryPieces> compute_pieces(const State &start,
                                               const State &end);

class TrajectoryComposite : public Trajectory {
public:
  TrajectoryComposite(const std::shared_ptr<const Scene> &scene,
                      const TrajectoryPieces &pieces) {
    // TODO convert pieces into trajectories
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

TrajectoryPtr CartPosesConnector::getTrajectory(const State &start,
                                                const State &end) const {
  const auto pieces = compute_pieces(start, end);
  if (pieces) {
    return std::make_unique<TrajectoryComposite>(scene, pieces.value());
  }
  return nullptr;
}

float CartPosesConnector::minCost2Go(const State &start,
                                     const State &end) const {
  const auto pieces = compute_pieces(start, end);
  if (pieces) {
    float result = 0;
    if (pieces->line_start) {
      result += distance(pieces->line_start->start.data(),
                         pieces->line_start->end.data());
    }
    if (pieces->arc) {
      result += pieces->arc->ray *
                std::abs(pieces->arc->angle_end - pieces->arc->angle_start);
    }
    if (pieces->line_end) {
      result += distance(pieces->line_end->start.data(),
                         pieces->line_end->end.data());
    }
    return result;
  }
  return COST_MAX;
}
} // namespace mt_rrt::samples