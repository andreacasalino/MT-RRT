/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/PlanningProblem.h>
#include <MT-RRT-core/TunneledConnector.h>

namespace mt_rrt::utils {
struct Box {
  State min_corner;
  State max_corner;
};
using Boxes = std::vector<Box>;

bool collides(const State &segment_start, const State &segment_end,
              const Box &box);

Boxes make_random_boxes(const std::size_t clusters,
                        const std::size_t boxes_numb);

// Unvierse is a (-1 , ... , -1) x (1 , ... , 1) hyperbox, with steer radius
// equal to 0.05
class PointConnector : public TunneledConnector {
public:
  PointConnector(const std::size_t size);

  PointConnector(const Boxes &obstacles);

  PointConnector(const PointConnector &o);

  const Boxes &getBoxes() const { return *obstacles; }

  std::unique_ptr<Connector> copy() const final {
    return std::make_unique<PointConnector>(*this);
  }

  static const float STEER_DEGREE;

protected:
  bool checkAdvancement(const State &previous_state,
                        const State &advanced_state) const override;

  using BoxesPtr = std::shared_ptr<const Boxes>;
  BoxesPtr obstacles;
};

template <typename... PointConnectorArgs>
std::shared_ptr<ProblemDescription>
make_point_problem(const std::optional<Seed> &seed,
                   PointConnectorArgs... args) {
  std::unique_ptr<PointConnector> connector = std::make_unique<PointConnector>(
      std::forward<PointConnectorArgs>(args)...);

  State min_corner, max_corner;
  min_corner.reserve(connector->getStateSpaceSize());
  max_corner.reserve(connector->getStateSpaceSize());
  for (std::size_t k = 0; k < connector->getStateSpaceSize(); ++k) {
    min_corner.push_back(-1.f);
    max_corner.push_back(1.f);
  }
  std::unique_ptr<HyperBox> sampler =
      std::make_unique<HyperBox>(min_corner, max_corner, seed);

  std::shared_ptr<ProblemDescription> result;
  result.reset(new ProblemDescription{std::move(sampler), std::move(connector),
                                      true, Positive<float>{10.f}});
  return result;
}
} // namespace mt_rrt::utils
