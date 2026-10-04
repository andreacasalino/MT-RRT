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

#include <JsonConversions.h>
#include <LogResult.h>
#include <Primitives.h>

namespace mt_rrt {
namespace trivial_problem {
using BoxesPtr = std::shared_ptr<const geom::Boxes>;

struct Checker {
  Checker(BoxesPtr boxes) : boxes_{boxes} {};

  bool check_from_to(std::span<const float> from,
                     std::span<const float> to) const;

  auto getBoxes() const { return boxes_; }

private:
  BoxesPtr boxes_;
};

// Universe is a (-1 , -1) x (1 , 1) box, with steer radius
// equal to 0.05
class Connector : public EuclidianConnector<Checker>,
                  public Copiable<Connector> {
public:
  Connector(BoxesPtr boxes, SteerIterations steers);

  std::unique_ptr<Connector> copy() const final {
    return std::make_unique<Connector>(get().checker->getBoxes(), get().steers);
  }

  static const float STEER_DEGREE; // 0.05
};

namespace detail {
template <ExpansionStrategy ExpansionStrategyT>
using DescriptionBase =
    ProblemDescription<Connector, HyperBox, true, ExpansionStrategyT>;
}

template <ExpansionStrategy ExpansionStrategyT>
class Description : public detail::DescriptionBase<ExpansionStrategyT> {
  Description(const std::optional<Seed> &seed, geom::Boxes obstacles,
              SteerIterations steers)
      : detail::DescriptionBase<ExpansionStrategyT>{
            .gamma = 10.f,
            .sampler =
                std::make_unique<HyperBox>(std::vector<float>{-1.f, -1.f},
                                           std::vector<float>{1.f, 1.f}, seed),
            .connector = std::make_unique<Connector>(
                std::make_shared<geom::Boxes>(std::move(obstacles)), steers)} {}
};
} // namespace trivial_problem

template <ExpansionStrategy ExpansionStrategyT>
void to_json(LogResult &j,
             const trivial_problem::Description<ExpansionStrategyT> &subject) {
  to_json(j.addToScene("region"), geom::Box{{-1.f, -1.f}, {1.f, 1.f}});
  for (const auto &box : subject.getBoxes()) {
    j.addObstacle(box);
  }
}
} // namespace mt_rrt
