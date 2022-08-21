/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <PlanarRobotsProblem.h>

#include <JsonConvert.h>

namespace mt_rrt::samples {
void to_json(nlohmann::json &j, const Sphere &subject);
void to_json(nlohmann::json &j, const Robot &subject);
void to_json(nlohmann::json &j, const Scene &subject);

void from_json(const nlohmann::json &j, Sphere &subject);
void from_json(const nlohmann::json &j, Robot &subject);
void from_json(const nlohmann::json &j, Scene &subject);

class PlanarRobotsProblemConverter : public utils::ConverterT<PosesConnector> {
public:
  static const PlanarRobotsProblemConverter CONVERTER;

  std::shared_ptr<mt_rrt::ProblemDescription>
  fromJson(const std::optional<Seed> &seed,
           const nlohmann::json &content) const final;

  void toJson(nlohmann::json &recipient,
              const std::vector<State> &sol) const final;

protected:
  void toJson_(nlohmann::json &recipient,
               const PosesConnector &connector) const final;
};
} // namespace mt_rrt::samples
