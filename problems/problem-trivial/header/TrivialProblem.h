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

namespace mt_rrt::trivial {
// Universe is a (-1 , -1) x (1 , 1) box, with steer radius
// equal to 0.05
class TrivialProblemConnector : public TunneledConnector {
public:
  TrivialProblemConnector(const geom::Boxes &obstacles);

  TrivialProblemConnector(const TrivialProblemConnector &o);

  const geom::Boxes &getBoxes() const { return *obstacles; }

  std::unique_ptr<Connector> copy() const final {
    return std::make_unique<TrivialProblemConnector>(*this);
  }

  static const float STEER_DEGREE; // 0.05

  static std::shared_ptr<ProblemDescription>
  make(const std::optional<Seed> &seed, const geom::Boxes &obstacles);

protected:
  bool checkAdvancement(const View &previous_state,
                        const View &advanced_state) const override;

  using BoxesPtr = std::shared_ptr<const geom::Boxes>;
  BoxesPtr obstacles;
};
} // namespace mt_rrt::trivial
