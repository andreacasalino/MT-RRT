/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/ProblemDescription.h>
#include <MT-RRT/TunneledConnector.h>
#include <MT-RRT/Types.h>

#include <Primitives.h>

#include <array>
#include <variant>

namespace mt_rrt::navigation {
class CartSteerLimits {
public:
  CartSteerLimits(float min_radius = 0.1f, float max_radius = 1.f);

  float minRadius() const { return min_steer_radius.get(); }
  float maxRadius() const { return max_steer_radius.get(); }

private:
  Positive<float> min_steer_radius;
  Positive<float> max_steer_radius;
};

// frame attached to cart has origin in the cart baricenter:
//
//  <--------> width
//  ----------   ^
//  |        |   |
//  |    x   |   |
//  |    ^   |   |
//  |    |   |   |  length
//  | y <-   |   |
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
  // [x_baricenter, y_baricenter, orientation -> angle of x axis w.r.t. the
  // absolute horizontal x ]
  bool isCollisionPresent(const geom::Sphere &obstacle,
                          const View &cart_state) const;

  const CartSteerLimits &steerLimits() const { return steer_limits; }

  float getWidth() const { return width.get(); }
  float getLength() const { return length.get(); }

private:
  Positive<float> width;
  Positive<float> length;
  CartSteerLimits steer_limits;

  std::array<geom::Point, 4> cart_perimeter;
};

struct Scene {
  Cart cart;
  std::vector<geom::Sphere> obstacles;
};

struct TrivialLine {
  View start;
  View end;
};

struct Blended {
  View start;
  View end;
  float ray;
  geom::Point center;
  geom::Point arc_begin;
  geom::Point arc_end;
};

using CartTrajectoryInfo = std::variant<Blended, TrivialLine>;

std::optional<CartTrajectoryInfo>
compute_cart_trajectory_info(const View &start, const View &end,
                             const CartSteerLimits &steer_limits);

class CartPosesConnector : public TunneledConnector {
public:
  CartPosesConnector(const Scene &scene);
  CartPosesConnector(const CartPosesConnector &o)
      : CartPosesConnector(*o.scene) {}

  std::shared_ptr<const Scene> scene;

  ConnectorPtr copy() const override {
    return std::make_unique<CartPosesConnector>(*this);
  }
  float minCost2Go(const View &start, const View &end) const override;
  TrajectoryPtr getTrajectory(const View &start,
                              const View &end) const override;

  static const float STEER_DEGREE;

  // workspace is assumed equal to bounding box from [-5,-5] to [5,5]
  static std::shared_ptr<ProblemDescription>
  make(const std::optional<Seed> &seed, const Scene &scene);

protected:
  class CartTrajectory;

  class CartLine;

  bool checkAdvancement(const View &, const View &advanced_state) const final;
};

} // namespace mt_rrt::navigation
