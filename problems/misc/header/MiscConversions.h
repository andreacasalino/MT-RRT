/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Node.h>
#include <MT-RRT/TreeHandler.h>

#include <Geometry.h>
#include <Primitives.h>

#include <nlohmann/json.hpp>

namespace mt_rrt {
void to_json(nlohmann::json &j, const std::vector<float> &subject);

void to_json(nlohmann::json &j, const View &subject);

void to_json(nlohmann::json &j, const geom::Point &subject);

void to_json(nlohmann::json &j, const geom::Versor &subject);

void to_json(nlohmann::json &j, const geom::Sphere &subject);

void to_json(nlohmann::json &j, const geom::Segment &subject);

void to_json(nlohmann::json &j, const geom::Transform &subject);

void to_json(nlohmann::json &j, const geom::Box &subject);

void to_json(nlohmann::json &j, const TreeHandler &subject);

template <typename ConnectorT>
std::vector<std::vector<float>>
extract_solution(const ConnectorT &connector,
                 const std::vector<std::vector<float>> &solution) {
  return solution;
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

geom::Point from_json_point(const nlohmann::json &src);

geom::Transform from_json_transform(const nlohmann::json &src);

geom::Sphere from_json_sphere(const nlohmann::json &src);
} // namespace mt_rrt
