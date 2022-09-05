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

void to_json(nlohmann::json& j, const Sphere& subject);

void from_json(const nlohmann::json &j, std::vector<Sphere> &obstacles);

//class TrivialProblemConverter
//    : public utils::ConverterT<TrivialProblemConnector> {
//public:
//  static const TrivialProblemConverter CONVERTER;
//
//  std::shared_ptr<ProblemDescription>
//  fromJson(const std::optional<Seed> &seed,
//           const nlohmann::json &content) const final;
//
//protected:
//  void toJson_(nlohmann::json &recipient,
//               const TrivialProblemConnector &connector) const final;
//};
} // namespace mt_rrt::samples
