/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/PlanningProblem.h>
#include <MT-RRT-core/TunneledConnector.h>

#include <JsonConvert.h>

namespace mt_rrt::samples {
struct Box {
  State min_corner;
  State max_corner;
};
using Boxes = std::vector<Box>;

bool collides(const State &segment_start, const State &segment_end,
              const Box &box);

// Unvierse is a (-1 , ... , -1) x (1 , ... , 1) hyperbox, with steer radius
// equal to 0.05
class TrivialProblemConnector : public TunneledConnector {
public:
  TrivialProblemConnector(const std::size_t size);

  TrivialProblemConnector(const Boxes &obstacles);

  TrivialProblemConnector(const TrivialProblemConnector &o);

  const Boxes &getBoxes() const { return *obstacles; }

  std::unique_ptr<Connector> copy() const final {
    return std::make_unique<TrivialProblemConnector>(*this);
  }

  // equal to 0.05
  static const float STEER_DEGREE;

protected:
  bool checkAdvancement(const State &previous_state,
                        const State &advanced_state) const override;

  using BoxesPtr = std::shared_ptr<const Boxes>;
  BoxesPtr obstacles;
};

SamplerPtr make_trivial_problem_sampler(std::size_t space_size,
                                        const std::optional<Seed> &seed);

template <typename... ConnectorArgs>
std::shared_ptr<ProblemDescription>
make_trivial_problem_description(const std::optional<Seed> &seed,
                                 ConnectorArgs... args) {
  auto connector = std::make_unique<TrivialProblemConnector>(
      std::forward<ConnectorArgs>(args)...);

  std::shared_ptr<ProblemDescription> result;
  result.reset(new ProblemDescription{
      make_trivial_problem_sampler(connector->getStateSpaceSize(), seed),
      std::move(connector), true, Positive<float>{10.f}});
  return result;
}

void to_json(nlohmann::json &j, const Box &subject);

class TrivialProblemConnectorLogger
    : public utils::ConnectorLoggerTyped<TrivialProblemConnector> {
public:
  static const TrivialProblemConnectorLogger LOGGER;

protected:
  void log(nlohmann::json &j, const TrivialProblemConnector &c) const final;
};
} // namespace mt_rrt::samples
