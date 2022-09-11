/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <NavigationProblem.h>

#include <JsonConvert.h>

namespace mt_rrt::samples {
void to_json(nlohmann::json &j, const Cart &subject);

void to_json(nlohmann::json &j, const Sphere &subject);

void to_json(nlohmann::json &j, const Scene &scene);

void from_json(const nlohmann::json &j, std::unique_ptr<Scene> &scene);

class NavigationProblemConverter
    : public utils::ConverterT<CartPosesConnector> {
public:
  static const NavigationProblemConverter CONVERTER;

  std::shared_ptr<ProblemDescription>
  fromJson(const std::optional<Seed> &seed,
           const nlohmann::json &content) const final;

  void toJson(nlohmann::json &recipient, const std::vector<State> &sol,
              const Connector &connector) const final;

protected:
  void toJson_(nlohmann::json &recipient,
               const CartPosesConnector &connector) const final;
};
} // namespace mt_rrt::samples
