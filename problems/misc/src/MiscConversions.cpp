/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MiscConversions.h>

namespace mt_rrt {
void to_json(nlohmann::json &j, const std::vector<float> &subject) {
  j = nlohmann::json::array();
  for (auto val : subject) {
    j.emplace_back(val);
  }
}

void to_json(nlohmann::json &j, const View &subject) {
  to_json(j, subject.convert());
}

void to_json(nlohmann::json &j, const geom::Point &subject) {
  j = nlohmann::json::array();
  j.emplace_back(subject.data()[0]);
  j.emplace_back(subject.data()[1]);
}

void to_json(nlohmann::json &j, const geom::Versor &subject) {
  j["angle"] = subject.angle();
  j["cos"] = subject.cos();
  j["sin"] = subject.sin();
}

void to_json(nlohmann::json &j, const geom::Sphere &subject) {
  to_json(j["center"], subject.center);
  j["ray"] = subject.ray.get();
}

void to_json(nlohmann::json &j, const geom::Segment &subject) {
  to_json(j["start"], subject.getStart());
  to_json(j["end"], subject.getEnd());
}

void to_json(nlohmann::json &j, const geom::Transform &subject) {
  j["angle"] = subject.getAngle();
  to_json(j["traslation"], subject.getTraslation());
}

void to_json(nlohmann::json &j, const geom::Box &subject) {
  to_json(j["min"], subject.min_corner);
  to_json(j["max"], subject.max_corner);
  if (subject.trsf.has_value()) {
    to_json(j["transform"], subject.trsf.value());
  }
}

void to_json(nlohmann::json &j, const PlannerSolution::TreeSerialized &subject) {
  j = nlohmann::json::array();
  for (const auto& node : subject) {
    auto &added = j.emplace_back();
    added["state"] = node.state;
    added["from"] = subject[node.parent_index].state;
    added["cost2Go"] = node.cost2Go;
  }
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

geom::Point from_json_point(const nlohmann::json &src) {
  return geom::Point{src[0], src[1]};
}

geom::Transform from_json_transform(const nlohmann::json &src) {
  std::optional<float> angle;
  if (src.contains("angle")) {
    float tmp = src["angle"];
    tmp = tmp * geom::PI / 180.f;
    angle.emplace(tmp);
  }
  std::optional<geom::Point> translation;
  if (src.contains("translation")) {
    translation = from_json_point(src["translation"]);
  }
  std::optional<geom::Transform> transform;
  if (src.contains("pivot")) {
    if (!angle.has_value()) {
      throw Error{"The pivot was specified but an angle no"};
    }
    geom::Point pivot = from_json_point(src["pivot"]);
    transform = geom::Transform::rotationAroundCenter(angle.value(), pivot);
    if (translation.has_value()) {
      transform = geom::Transform::combine(
          geom::Transform{std::nullopt, translation}, transform.value());
    }
  } else {
    transform.emplace(angle, translation);
  }
  return transform.value();
}

geom::Sphere from_json_sphere(const nlohmann::json &src) {
  float ray = src["ray"];
  auto center = from_json_point(src["center"]);
  return geom::Sphere{ray, center};
}

} // namespace mt_rrt
