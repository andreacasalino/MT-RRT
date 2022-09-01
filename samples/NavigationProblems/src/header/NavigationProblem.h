/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/PlanningProblem.h>
#include <MT-RRT-core/TunneledConnector.h>

#include <array>
#include <variant>

namespace mt_rrt::samples {
static constexpr float PI = 3.1415926535f;
static constexpr float PI_HALF = 0.5f * PI;

float to_rad(float angle);

float to_grad(float angle);

using Point = std::array<float, 2>;

struct Sphere {
  Positive<float> ray;
  Point center;
};

class CartSteerLimits {
public:
  CartSteerLimits(float min_radius, float max_radius);

  float minRadius() const { return min_steer_radius.get(); }
  float maxRadius() const { return max_steer_radius.get(); }

private:
  Positive<float> min_steer_radius;
  Positive<float> max_steer_radius;
};

// frame attached to cart has an origin in the cart baricenter:
//
//  <--------> width
//  ----------   ^
//  |        |   |
//  |    y   |   |
//  |    ^   |   |
//  |    |   |   |  length
//  |    -> x|   |
//  |        |   |
//  |        |   |
//  |        |   |
//  |        |   |
//  ----------   ^
//
class Cart {
public:
  Cart(float width, float length, const CartSteerLimits &steer_limits);

  // cart_state assumed formatted in this way:
  // [x_baricenter, y_baricenter, orientation]
  bool isCollisionPresent(const Sphere &obstacle,
                          const State &cart_state) const;

  const CartSteerLimits &steerLimits() const { return steer_limits; }

private:
  Positive<float> width;
  Positive<float> length;
  CartSteerLimits steer_limits;

  std::array<Point, 4> cart_perimeter;
};

struct Scene {
  Cart cart;
  std::vector<Sphere> obstacles;
};

struct TrivialLine {
    const State& start;
    const State& end;
};
struct Blended {
    const State& start;
    const State& end;
    float ray;
    Point center;
    Point arc_begin;
    Point arc_end;
};
using TrajectoryInfo = std::variant<Blended, TrivialLine>;
std::optional<TrajectoryInfo>
compute_cart_trajectory_info(const State& start, const State& end,
    const CartSteerLimits& steer_limits);

class CartPosesConnector : public TunneledConnector {
public:
  CartPosesConnector(const Scene& scene);
  CartPosesConnector(const CartPosesConnector &o) : CartPosesConnector(*o.scene) {
  }

  std::shared_ptr<const Scene> scene = std::make_shared<Scene>();

  ConnectorPtr copy() const override {
    return std::make_unique<CartPosesConnector>(*this);
  }
  float minCost2Go(const State &start, const State &end) const override;
  TrajectoryPtr getTrajectory(const State &start,
                              const State &end) const override;

  static const float STEER_DEGREE;

protected:
    class TrajectoryComposite;

    bool checkAdvancement(const State& previous_state,
        const State& advanced_state) const final;
};

std::shared_ptr<ProblemDescription>
make_problem_description(const std::optional<Seed> &seed, const Scene &scene);
} // namespace mt_rrt::samples
