/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <PlanarRobotsProblem.h>

#include <IO.h>

namespace mt_rrt::samples {
void to_json(nlohmann::json &j, const Sphere &subject);
void to_json(nlohmann::json &j, const Robot &subject);
void to_json(nlohmann::json &j, const Scene &subject);

void from_json(const nlohmann::json &j, Sphere &subject);
void from_json(const nlohmann::json &j, Robot &subject);
void from_json(const nlohmann::json &j, Scene &subject);

class PlanarRobotsProblemConverter : public utils::Converter {
public:
  static const PlanarRobotsProblemConverter CONVERTER;

  void fromJson(const nlohmann::json& json,
      ProblemDescription& description) const final;

  void toJson(nlohmann::json& json,
      const ProblemDescription& description) const final;

  std::vector<State>
      interpolate(const ProblemDescription& description,
          const std::vector<State>& solution) const final;
};
} // namespace mt_rrt::samples
