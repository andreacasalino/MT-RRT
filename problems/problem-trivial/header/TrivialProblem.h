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

struct Checker : public Copiable<Checker> {
  Checker(BoxesPtr boxes) : boxes_{boxes} {};

  bool check_from_to(std::span<const float> from,
                     std::span<const float> to) const;

  auto getBoxes() const { return boxes_; }

  std::unique_ptr<Checker> copy() const final {
    return std::make_unique<Checker>(boxes_);
  }

private:
  BoxesPtr boxes_;
};

namespace detail {
template <ExpansionStrategy ExpansionStrategyT>
using DescriptionBase = ProblemDescription<EuclidianConnector<Checker>,
                                           HyperBox, true, ExpansionStrategyT>;
}

// Universe is a (-1 , -1) x (1 , 1) box, with steer radius
// equal to 0.05
template <ExpansionStrategy ExpansionStrategyT>
class Description : public detail::DescriptionBase<ExpansionStrategyT> {
public:
  static const inline float STATE_BOX_DIAGONAL_LENGTH = 2.f * sqrtf(1.f);

  static const inline Positive STEER_DEGREE =
      Positive{STATE_BOX_DIAGONAL_LENGTH / 15.f};

  Description(const std::optional<Seed> &seed, geom::Boxes obstacles,
              SteerIterations steers)
      : detail::DescriptionBase<ExpansionStrategyT>{
            .gamma = 10.f,
            .sampler =
                std::make_unique<HyperBox>(std::vector<float>{-1.f, -1.f},
                                           std::vector<float>{1.f, 1.f}, seed),
            .connector = std::make_unique<EuclidianConnector<Checker>>(
                make_checker(std::move(obstacles)), STEER_DEGREE, steers)} {}

private:
  static std::unique_ptr<Checker> make_checker(geom::Boxes obstacles) {
    return std::make_unique<Checker>(
        std::make_shared<geom::Boxes>(std::move(obstacles)));
  }
};
} // namespace trivial_problem

void to_json(LogResult &j, const trivial_problem::Checker &subject);
} // namespace mt_rrt
