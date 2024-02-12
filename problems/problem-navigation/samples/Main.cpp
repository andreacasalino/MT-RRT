#include <NavigationProblem.h>
#include <NavigationProblemConversions.h>
#include <SampleBase.h>

namespace mt_rrt {
template <>
std::vector<std::vector<float>>
extract_solution<navigation::CartPosesConnector>(
    const navigation::CartPosesConnector &connector,
    const std::vector<std::vector<float>> &solution) {
  return interpolate(connector, solution);
}

template <>
ProblemDescription
from_json<navigation::CartPosesConnector>(const nlohmann::json &src) {
  unsigned seed = src["seed"];

  const float w = src["cart"]["width"];
  const float l = src["cart"]["length"];
  const float steer_min = src["cart"]["steer"]["min"];
  const float steer_max = src["cart"]["steer"]["max"];

  navigation::Scene scene = navigation::Scene{navigation::Cart{
      w, l, navigation::CartSteerLimits{steer_min, steer_max}}};
  for (const auto &obstacle_json : src["obstacles"]) {
    scene.obstacles.emplace_back(from_json_sphere(obstacle_json));
  }
  return std::move(*navigation::CartPosesConnector::make(Seed{seed}, scene));
}

struct PoseParser {
  std::vector<float> operator()(std::vector<float> subject) const {
    subject[2] *= geom::PI / 180.f;
    return subject;
  }
};
} // namespace mt_rrt

int main(int argc, const char **argv) {
  mt_rrt::SampleBase<mt_rrt::navigation::CartPosesConnector> sample{argc, argv,
                                                                    "Carts"};

  sample.process<mt_rrt::PoseParser>();

  return EXIT_SUCCESS;
}
