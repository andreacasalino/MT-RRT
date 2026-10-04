/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/EuclidianConnector.h>
#include <MT-RRT/HyperBox.h>
#include <MT-RRT/ProblemDescription.h>

#include <Primitives.h>

namespace mt_rrt::trivial {
using BoxesPtr = std::shared_ptr<const geom::Boxes>;

struct TrivialProblemChecker {
  TrivialProblemChecker(BoxesPtr boxes) : boxes_{boxes} {};

  bool check_from_to(std::span<const float> from,
                     std::span<const float> to) const;

  auto getBoxes() const { return boxes_; }

private:
  BoxesPtr boxes_;
};

// Universe is a (-1 , -1) x (1 , 1) box, with steer radius
// equal to 0.05
class TrivialProblemConnector
    : public EuclidianConnector<TrivialProblemChecker>,
      public Copiable<TrivialProblemConnector> {
public:
  TrivialProblemConnector(BoxesPtr boxes, SteerIterations steers);

  std::unique_ptr<TrivialProblemConnector> copy() const final {
    return std::make_unique<TrivialProblemConnector>(get().checker->getBoxes(),
                                                     get().steers);
  }

  static const float STEER_DEGREE; // 0.05
};

template <ExpansionStrategy ExpansionStrategyT>
using TrivialProblemDescription =
    ProblemDescription<TrivialProblemConnector, HyperBox, true,
                       ExpansionStrategyT>;

template <ExpansionStrategy ExpansionStrategyT>
TrivialProblemDescription<ExpansionStrategyT>
make_problem(const std::optional<Seed> &seed, geom::Boxes obstacles,
             SteerIterations steers) {
  return TrivialProblemDescription<ExpansionStrategyT>{
      .gamma = 10.f,
      .sampler = std::make_unique<HyperBox>({-1.f, -1.f}, {1.f, 1.f}, seed),
      .connector = std::make_unique<TrivialProblemConnector>(
          std::make_shared<geom::Boxes>(std::move(obstacles)), steer)};
}
} // namespace mt_rrt::trivial
