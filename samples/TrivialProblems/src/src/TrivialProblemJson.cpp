/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Geometry.h>
#include <TrivialProblemJson.h>

#include <optional>

namespace mt_rrt::samples {
void to_json(nlohmann::json &j, const Transform &subject) {
  j["angle"] = utils::to_grad(subject.getAngle());
  j["traslation"] = subject.getTraslation();
}

void to_json(nlohmann::json &j, const Box &subject) {
  j["box"]["min"] = subject.min_corner;
  j["box"]["max"] = subject.max_corner;
  if (subject.trsf) {
    j["transform"] = subject.trsf.value();
  }
}

namespace {
Transform import_transform(const nlohmann::json &j) {
  float rot = 0;
  std::optional<Transform::Traslation> center;
  if (j.contains("angle")) {
    rot = j["angle"];
    rot = utils::to_rad(rot);
    if (j.contains("center")) {
      center.emplace() = j["center"];
    }
  }

  Transform::Traslation trasl = {0, 0};
  if (j.contains("traslation")) {
    trasl = j["traslation"];
  }

  if (center) {
    auto result = Transform::rotationAroundCenter(rot, center.value());
    result = Transform::combine(Transform{std::nullopt, trasl}, result);
    return result;
  }

  return Transform{rot, trasl};
}

Box import_box(const nlohmann::json &j,
               const std::unordered_map<std::string, Box *> &labeled) {
  State min, max;
  std::optional<Transform> trsf;

  if (j.contains("box")) {
    auto &j_box = j["box"];
    State min_temp = j_box["min"];
    State max_temp = j_box["max"];
    min = std::move(min_temp);
    max = std::move(max_temp);
  } else if (j.contains("copy")) {
    const std::string box_to_copy_label = j["copy"]["source"];
    auto labeled_it = labeled.find(box_to_copy_label);
    if (labeled_it == labeled.end()) {
      throw Error{"Unabled to find box labelled ", box_to_copy_label};
    }
    min = labeled_it->second->min_corner;
    max = labeled_it->second->max_corner;
    if (labeled_it->second->trsf) {
      trsf = labeled_it->second->trsf.value();
    }
  }

  if (j.contains("transform")) {
    auto additional_trsf = import_transform(j["transform"]);
    if (trsf) {
      trsf = Transform::combine(additional_trsf, trsf.value());
    } else {
      trsf = additional_trsf;
    }
  }

  return Box{min, max, trsf};
}
} // namespace

void from_json(const nlohmann::json &j, Boxes &boxes) {
  std::unordered_map<std::string, Box *> labeled;
  std::list<std::unique_ptr<Box>> boxes_list;
  for (const auto &box_json : j) {
    auto &added = boxes_list.emplace_back(
        std::make_unique<Box>(import_box(box_json, labeled)));
    if (box_json.contains("label")) {
      std::string label = box_json["label"];
      if (labeled.find(label) != labeled.end()) {
        throw Error{"Found multiple boxes labeled as ", label};
      }
      labeled.emplace(std::move(label), added.get());
    }
  }
  for (auto &box : boxes_list) {
    boxes.emplace_back(*box);
  }
}

const TrivialProblemConverter TrivialProblemConverter::CONVERTER =
    TrivialProblemConverter{};

void TrivialProblemConverter::fromJson(const nlohmann::json& json,
    ProblemDescription& description) const {
    Boxes boxes;
    from_json(json["Boxes"], boxes);
    std::optional<mt_rrt::Seed> seed;
    if (json.contains("seed")) {
        mt_rrt::Seed seed_value = json["seed"];
        seed.emplace(seed_value);
    }
    auto desc = make_trivial_problem_description(seed, boxes);
    description.connector = std::move(desc->connector);
    description.sampler = std::move(desc->sampler);
    description.simmetry = true;
    description.gamma = desc->gamma;
}

void TrivialProblemConverter::toJson(nlohmann::json& json,
    const ProblemDescription& description) const {
    json["Boxes"] = static_cast<const TrivialProblemConnector&>(*description.connector).getBoxes();
}
} // namespace mt_rrt::samples
