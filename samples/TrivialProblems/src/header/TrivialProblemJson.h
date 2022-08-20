/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <TrivialProblem.h>

#include <JsonConvert.h>

namespace mt_rrt::samples {
void to_json(nlohmann::json &j, const Transform &subject);

void to_json(nlohmann::json &j, const Box &subject);

// The passed json is expected to be an array of elements[e0, ..., eN].
// Each element should have the following format:
//  - e["label"] (optional) -> a name to associate to this box
//  - one of:
//     - e["box"]
//        - e["box"]["min"] -> min corner of the AABB defining the box
//        - e["box"]["max"] -> max corner of the AABB defining the box
//     - e["copy"] -> clone of an already defined box
//        - e["copy"]["source"] -> label of the box to clone
//  - e["transform"] (optional) -> transformation to apply to the box
//     - e["transform"]["rotation"] -> rotation angle
//     - e["transform"]["traslation"] -> {x, y}
//     - e["transform"]["center"] (optional) -> if ["rotation"] was defined,
//     prescribes the rotation center for the rotation
void from_json(const nlohmann::json &j, Boxes &boxes);

class TrivialProblemConverter
    : public utils::ConverterT<TrivialProblemConnector> {
public:
  static const TrivialProblemConverter CONVERTER;

  ProblemDescription fromJson(const std::optional<Seed> &seed,
                              const nlohmann::json &content) const final;

protected:
  void toJson_(nlohmann::json &recipient,
               const TrivialProblemConnector &connector) const final;
};
} // namespace mt_rrt::samples
