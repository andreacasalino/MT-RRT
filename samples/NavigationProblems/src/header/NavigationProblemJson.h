/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <NavigationProblem.h>

#include <IO.h>

namespace mt_rrt::samples {
void to_json(nlohmann::json &j, const Cart &subject);

void to_json(nlohmann::json &j, const Sphere &subject);

void to_json(nlohmann::json &j, const Scene &scene);

void from_json(const nlohmann::json &j, std::unique_ptr<Scene> &scene);

class NavigationProblemConverter
    : public utils::Converter {
public:
  static const NavigationProblemConverter CONVERTER;

  void fromJson(const nlohmann::json& json,
      ProblemDescription& description) const final;

  void toJson(nlohmann::json& json,
      const ProblemDescription& description) const final;

  std::vector<State>
      interpolate(const ProblemDescription& description,
          const std::vector<State>& solution) const final;
};
} // namespace mt_rrt::samples
