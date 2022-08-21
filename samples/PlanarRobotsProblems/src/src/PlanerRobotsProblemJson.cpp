/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <PlanerRobotsProblemJson.h>

#include <fstream>

namespace mt_rrt::samples {
namespace {
void to_rad_state(State &subject) {
  for (auto &val : subject) {
    val = to_rad(val);
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

const PlanarRobotsProblemConverter PlanarRobotsProblemConverter::CONVERTER =
    PlanarRobotsProblemConverter{};

std::shared_ptr<mt_rrt::ProblemDescription>
PlanarRobotsProblemConverter::fromJson(const std::optional<Seed> &seed,
                                       const nlohmann::json &content) const {
  Scene scene;
  from_json(content, scene);
  return make_problem_description(seed, scene);
}

void PlanarRobotsProblemConverter::toJson_(
    nlohmann::json &recipient, const PosesConnector &connector) const {
  to_json(recipient, *connector.scene);
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

static const float MAX_JOINT_DISTANCE = mt_rrt::samples::to_rad(15);

std::vector<mt_rrt::State> interpolate(const mt_rrt::State &start,
                                       const mt_rrt::State &end) {
  std::size_t intervals = static_cast<std::size_t>(
      std::ceil(mt_rrt::samples::euclidean_distance(start.data(), end.data(),
                                                    start.size()) /
                MAX_JOINT_DISTANCE));
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

void PlanarRobotsProblemConverter::toJson(nlohmann::json &recipient,
                                          const std::vector<State> &sol) const {
  std::vector<mt_rrt::State> interpolated;
  interpolated.push_back(sol.front());
  for (std::size_t k = 1; k < sol.size(); ++k) {
    auto interpolated_segment = interpolate(sol[k - 1], sol[k]);
    interpolated.insert(interpolated.end(), interpolated_segment.begin(),
                        interpolated_segment.end());
  }
  recipient = interpolated;
}
} // namespace mt_rrt::samples
