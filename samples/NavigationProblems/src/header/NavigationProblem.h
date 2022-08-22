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

namespace mt_rrt::samples {
static constexpr float PI = 3.1415926535f;

float to_rad(float angle);

float to_grad(float angle);

using Point = std::array<float, 2>;

struct Sphere {
  Positive<float> ray;
  Point center;
};

// Cart State assumes [x, y, angle]
class Cart {
public:
  Cart(float width, float length);

  bool isCollisionPresent(const Cart &cart, const Sphere &obstacle,
                          const State &cart_state) const;

private:
  Positive<float> width;
  Positive<float> length;

  std::array<Point, 4> cart_perimeter;
};

struct Scene {
  Cart cart;
  std::vector<Sphere> obstacles;
};

class CartPosesConnector : public Connector {
public:
  CartPosesConnector() = default;
  CartPosesConnector(const CartPosesConnector &o) {
    scene = std::make_shared<Scene>(*o.scene);
  }

  std::shared_ptr<const Scene> scene = std::make_shared<Scene>();

  ConnectorPtr copy() const override {
    return std::make_unique<CartPosesConnector>(*this);
  }
  float minCost2Go(const State &start, const State &end) const override;
  TrajectoryPtr getTrajectory(const State &start,
                              const State &end) const override;
};

std::shared_ptr<ProblemDescription>
make_problem_description(const std::optional<Seed> &seed, const Scene &scene);
} // namespace mt_rrt::samples
