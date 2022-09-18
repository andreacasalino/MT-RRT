/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <NavigationProblemJson.h>

namespace mt_rrt::samples {
void to_json(nlohmann::json &j, const Cart &subject) {
  j["width"] = subject.getWidth();
  j["length"] = subject.getLength();
  j["steer"]["min"] = subject.steerLimits().minRadius();
  j["steer"]["max"] = subject.steerLimits().maxRadius();
}

void to_json(nlohmann::json &j, const Sphere &subject) {
  j["ray"] = subject.ray.get();
  j["center"] = subject.center;
}

void to_json(nlohmann::json &j, const Scene &scene) {
  j["cart"] = scene.cart;
  auto &obstacles_json = j["obstacles"];
  obstacles_json = nlohmann::json::array();
  for (const auto &sphere : scene.obstacles) {
    to_json(obstacles_json.emplace_back(), sphere);
  }
}

void from_json(const nlohmann::json &j, std::unique_ptr<Scene> &scene) {
  const float w = j["cart"]["width"];
  const float l = j["cart"]["length"];
  const float steer_min = j["cart"]["steer"]["min"];
  const float steer_max = j["cart"]["steer"]["max"];

  scene = std::make_unique<Scene>(
      Scene{Cart{w, l, CartSteerLimits{steer_min, steer_max}}, {}});
  for (const auto &obstacle_json : j["obstacles"]) {
    const float ray = obstacle_json["ray"];
    const Point center = obstacle_json["center"];
    scene->obstacles.emplace_back(Sphere{ray, std::move(center)});
  }
}

const NavigationProblemConverter NavigationProblemConverter::CONVERTER =
    NavigationProblemConverter{};

void NavigationProblemConverter::fromJson(const nlohmann::json& json,
    ProblemDescription& description) const {
  std::unique_ptr<Scene> scene;
  from_json(json["Scene"], scene);
  std::optional<mt_rrt::Seed> seed;
  if (json.contains("seed")) {
      mt_rrt::Seed seed_value = json["seed"];
      seed.emplace(seed_value);
  }
  auto desc = make_problem_description(seed, *scene);
  description.connector = std::move(desc->connector);
  description.sampler = std::move(desc->sampler);
  description.simmetry = true;
  description.gamma = desc->gamma;
}

void NavigationProblemConverter::toJson(nlohmann::json& json,
    const ProblemDescription& description) const {
    json["Scene"] = *static_cast<const CartPosesConnector&>(*description.connector).scene;
}

std::vector<State>
NavigationProblemConverter::interpolate(const ProblemDescription& description,
    const std::vector<State>& solution) const {
    const CartPosesConnector* connector = dynamic_cast<const CartPosesConnector*>(description.connector.get());

    auto throw_exc = []() {
        throw Error{ "something went wrong logging solution found" };
    };

    auto prev = solution.begin();
    auto current = solution.begin() + 1;
    std::vector<mt_rrt::State> interpolated;
    for (; current != solution.end(); ++current, ++prev) {
        interpolated.push_back(*prev);
        auto traj = connector->getTrajectory(*prev, *current);
        if (nullptr == traj) {
            throw_exc();
        }
        while (true) {
            auto status = traj->advance();
            if (status == AdvanceInfo::blocked) {
                throw_exc();
            }
            if (status == AdvanceInfo::targetReached) {
                break;
            }
            interpolated.push_back(traj->getState());
        }
    }
    interpolated.push_back(solution.back());
    return interpolated;
}
} // namespace mt_rrt::samples
