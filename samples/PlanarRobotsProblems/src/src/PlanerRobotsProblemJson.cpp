/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Geometry.h>
#include <PlanerRobotsProblemJson.h>

#include <fstream>

namespace mt_rrt::samples {
namespace {
void to_rad_state(State &subject) {
  for (auto &val : subject) {
    val = utils::to_rad(val);
  }
}
} // namespace

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
  j["base"]["angle"] = subject_base.angle * 180.f / utils::PI;
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
    base.angle = utils::to_rad(base.angle);
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
      nlohmann::from_json(link_json.at("limits").at("min"), length);
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
  auto &obstacles_json = j["obstacles"];
  obstacles_json = nlohmann::json::array();
  for (const auto &sphere : subject.obstacles) {
    to_json(obstacles_json.emplace_back(), sphere);
  };
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

const PlanarRobotsProblemConverter PlanarRobotsProblemConverter::CONVERTER =
    PlanarRobotsProblemConverter{};

void
PlanarRobotsProblemConverter::fromJson(const nlohmann::json& json,
    ProblemDescription& description) const {
  Scene scene;
  from_json(json["Scene"], scene);
  std::optional<mt_rrt::Seed> seed;
  if (json.contains("seed")) {
      mt_rrt::Seed seed_value = json["seed"];
      seed.emplace(seed_value);
  }
  auto desc = make_problem_description(seed, scene);
  description.connector = std::move(desc->connector);
  description.sampler = std::move(desc->sampler);
  description.simmetry = true;
  description.gamma = desc->gamma;
}

void PlanarRobotsProblemConverter::toJson(nlohmann::json& json,
    const ProblemDescription& description) const {
  to_json(json["Scene"], *static_cast<const PosesConnector&>(*description.connector).scene);
}

namespace {
// return a + c(b-a)
mt_rrt::State linear_combination(const mt_rrt::State &a, const mt_rrt::State &b,
                                 float c) {
  mt_rrt::State result = a;
  for (std::size_t k = 0; k < a.size(); ++k) {
    result[k] += c * (b[k] - a[k]);
  }
  return result;
}

static const float MAX_JOINT_DISTANCE = mt_rrt::utils::to_rad(15);

std::vector<mt_rrt::State> interpolate_(const mt_rrt::State &start,
                                       const mt_rrt::State &end) {
  std::size_t intervals = static_cast<std::size_t>(
      std::ceil(utils::distance(start, end) / MAX_JOINT_DISTANCE));
  intervals = std::max<std::size_t>(1, intervals);
  const float delta_c = 1.f / static_cast<float>(intervals);
  float c = delta_c;
  std::vector<mt_rrt::State> result;
  for (std::size_t i = 1; i < intervals; ++i) {
    result.emplace_back(linear_combination(start, end, c));
    c += delta_c;
  }
  result.push_back(end);
  return result;
}
} // namespace

std::vector<State>
PlanarRobotsProblemConverter::interpolate(const ProblemDescription& description,
    const std::vector<State>& solution) const {
    std::vector<mt_rrt::State> interpolated;
    interpolated.push_back(solution.front());
    for (std::size_t k = 1; k < solution.size(); ++k) {
        auto interpolated_segment = interpolate_(solution[k - 1], solution[k]);
        interpolated.insert(interpolated.end(), interpolated_segment.begin(),
            interpolated_segment.end());
    }
    return interpolated;
}
} // namespace mt_rrt::samples
