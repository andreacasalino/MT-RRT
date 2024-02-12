/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MiscConversions.h>
#include <PlanarRobotsProblemConversions.h>

namespace mt_rrt {
void to_json(nlohmann::json &j, const robots::Robot &subject) {
  auto base = subject.getBase();
  j["base"]["angle"] = base.angle;
  to_json(j["base"]["center"], base.position);
  auto &joints = j["joints"];
  joints = nlohmann::json::array();
  for (const auto &joint : subject.getJoints()) {
    auto &added = joints.emplace_back();
    added["ray"] = joint.ray.get();
    added["length"] = joint.length.get();
  }
}

void to_json(LogResult &j, const robots::PosesConnector &subject) {
  nlohmann::json &robots = j.addToScene("robots");
  robots = nlohmann::json::array();
  for (const auto &robot : subject.scene->robots) {
    to_json(robots.emplace_back(), robot);
  }
  for (const auto &sphere : subject.scene->obstacles) {
    j.addObstacle(sphere);
  }
}
} // namespace mt_rrt
