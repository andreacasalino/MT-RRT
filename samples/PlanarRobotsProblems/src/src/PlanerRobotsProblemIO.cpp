/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "PlanerRobotsProblemIO.h"

#include <fstream>

namespace mt_rrt::samples {
namespace {
void to_rad_state(State &subject) {
  for (auto &val : subject) {
    val = to_rad(val);
  }
}
} // namespace

std::shared_ptr<ProblemDescription>
make_problem_description(const std::optional<Seed> &seed,
                         const std::string &scene_json_filename, State &start,
                         State &end) {
  nlohmann::json j;
  utils::from_file(j, scene_json_filename);
  Scene scene;
  from_json(j["scene"], scene);
  nlohmann::from_json(j["start"], start);
  to_rad_state(start);
  nlohmann::from_json(j["end"], end);
  to_rad_state(end);
  return make_problem_description(seed, scene);
}

void to_json(nlohmann::json &j, const Sphere &subject) {
  j["ray"] = subject.ray.get();
  j["center"] = subject.center;
}
void from_json(const nlohmann::json &j, Sphere &subject) {
  float ray;
  nlohmann::from_json(j.at("ray"), ray);
  subject.ray.set(ray);
  nlohmann::from_json(j.at("center"), subject.center);
}

void to_json(nlohmann::json &j, const Robot &subject) {
  const auto subject_base = subject.getBase();
  j["base"]["position"] = subject_base.position;
  j["base"]["angle"] = subject_base.angle * 180.f / PI;
  auto &j_links = j["links"];
  j_links = nlohmann::json::array();
  for (const auto &link : subject.getLinks()) {
    auto &link_json = j_links.emplace_back();
    link_json["ray"] = link.ray.get();
    link_json["length"] = link.length.get();
    link_json["limits"]["min"] = link.joint_limits.min();
    link_json["limits"]["max"] = link.joint_limits.max();
  }
}
void from_json(const nlohmann::json &j, Robot &subject) {
  Robot::Base base;
  if (j.contains("base")) {
    nlohmann::from_json(j.at("base").at("position"), base.position);
    nlohmann::from_json(j.at("base").at("angle"), base.angle);
    base.angle = to_rad(base.angle);
  }

  std::vector<Robot::Link> links;
  auto &links_json = j.at("links");
  if (!links_json.is_array()) {
    throw Error{"Invalid json"};
  }
  for (const auto &link_json : links_json) {
    float ray;
    nlohmann::from_json(link_json.at("ray"), ray);
    float length;
    nlohmann::from_json(link_json.at("length"), length);

    std::optional<JointLimits> limits;
    if (link_json.contains("limits")) {
      float min;
      nlohmann::from_json(link_json.at("limits").at("min"), length);
      float max;
      nlohmann::from_json(link_json.at("limits").at("max"), length);
    }

    auto &link = links.emplace_back();
    link.ray.set(ray);
    link.length.set(length);
    link.joint_limits = limits ? limits.value() : JointLimits{};
  }
  subject = Robot{links, base};
}

void to_json(nlohmann::json &j, const Scene &subject) {
  j["obstacles"] = subject.obstacles;
  j["robots"] = subject.robots;
}
void from_json(const nlohmann::json &j, Scene &subject) {
  subject.obstacles.clear();
  auto &obstacles = j.at("obstacles");
  if (!obstacles.is_array()) {
    throw Error{"Invalid json"};
  }
  for (const auto &obstacle : obstacles) {
    from_json(obstacle, subject.obstacles.emplace_back(Sphere{}));
  }

  subject.robots.clear();
  auto &robots = j.at("robots");
  if (!robots.is_array()) {
    throw Error{"Invalid json"};
  }
  for (const auto &robot : robots) {
    from_json(robot, subject.robots.emplace_back(Robot{}));
  }
}

const PosesConnectorLogger PosesConnectorLogger::LOGGER =
    PosesConnectorLogger{};
} // namespace mt_rrt::samples
