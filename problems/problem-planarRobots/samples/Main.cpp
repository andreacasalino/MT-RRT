#include <PlanarRobotsProblem.h>
#include <PlanarRobotsProblemConversions.h>
#include <SampleBase.h>

namespace mt_rrt {
std::pair<std::vector<robots::Joint>, robots::Robot::Base>
from_json_robot(const nlohmann::json &src) {
  auto res =
      std::make_pair(std::vector<robots::Joint>{}, robots::Robot::Base{});
  auto &[joints, base] = res;
  for (const auto &joint_json : src["links"]) {
    auto &added = joints.emplace_back();
    added.ray.set(joint_json["ray"]);
    added.length.set(joint_json["length"]);
  }
  if (src.contains("base")) {
    const auto &base_json = src["base"];
    if (base_json.contains("angle")) {
      base.angle = base_json["angle"];
      base.angle *= geom::PI / 180.f;
    }
    if (base_json.contains("position")) {
      base.position = from_json_point(base_json["position"]);
    }
  }
  return res;
}

template <>
ProblemDescription
from_json<robots::PosesConnector>(const nlohmann::json &src) {
  unsigned seed = src["seed"];
  robots::Scene scene;
  for (const auto &robot : src["robots"]) {
    auto &&[joints, base] = from_json_robot(robot);
    scene.robots.emplace_back(joints, base);
  }
  for (const auto &sphere : src["obstacles"]) {
    scene.obstacles.emplace_back(from_json_sphere(sphere));
  }
  return std::move(*robots::PosesConnector::make(Seed{seed}, scene));
}

struct PoseParser {
  std::vector<float> operator()(std::vector<float> subject) const {
    for (auto &angle : subject) {
      angle *= geom::PI / 180.f;
    }
    return subject;
  }
};
} // namespace mt_rrt

int main(int argc, const char **argv) {
  mt_rrt::SampleBase<mt_rrt::robots::PosesConnector> sample{argc, argv,
                                                            "PlanarRobots"};

  sample.process<mt_rrt::PoseParser>();

  return EXIT_SUCCESS;
}
