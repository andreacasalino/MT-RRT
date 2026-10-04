/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <JsonConversions.h>

namespace mt_rrt {
void to_json(nlohmann::json &j, const geom::Point &subject) {
  j = subject.data();
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

void to_json(nlohmann::json &j, const Nodes &subject) {
  j = nlohmann::json::array();
  for (const auto &node : subject.getNodes()) {
    auto &added = j.emplace_back();
    added["state"] = node.data().state;
    const auto *parent = node.data().parent;
    added["from"] = parent ? parent->data().state : node.data().state;
    added["cost2Go"] = node.data().cost2Go.get();
  }
}

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

geom::PointAllocated from_json_point(const nlohmann::json &src) {
  return geom::PointAllocated{src[0].get<float>(), src[1].get<float>()};
}

namespace {
geom::Transform from_json_transform_pivot(const nlohmann::json &src) {
  if (!src.contains("angle")) {
    throw Error{"The pivot was specified but an angle no"};
  }
  geom::Point pivot = from_json_point(src["pivot"]);
  auto transform =
      geom::Transform::rotationAroundCenter(src["angle"].get<float>(), pivot);
  if (src.contains("translation")) {
    transform = geom::Transform::combine(
        geom::Transform{geom::TransformBuilder{}.traslation(
            from_json_point(src["translation"]))},
        transform);
  }
  return transform;
}
} // namespace

geom::Transform from_json_transform(const nlohmann::json &src) {
  if (src.contains("pivot")) {
    return from_json_transform_pivot(src);
  }

  geom::TransformBuilder builder;
  if (src.contains("angle")) {
    float angle = src["angle"];
    angle = angle * geom::PI / 180.f;
    builder.angle(angle);
  }
  if (src.contains("translation")) {
    builder.traslation(from_json_point(src["translation"]));
  }
  return {builder};
}

geom::Sphere from_json_sphere(const nlohmann::json &src) {
  float ray = src["ray"].get<float>();
  auto center = from_json_point(src["center"]);
  return geom::Sphere{ray, center};
}

} // namespace mt_rrt
