/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <NavigationProblemJson.h>

#include <JsonConvert.h>

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
  auto obstacles_json = j["obstacles"];
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

std::shared_ptr<mt_rrt::ProblemDescription>
NavigationProblemConverter::fromJson(const std::optional<Seed> &seed,
                                     const nlohmann::json &content) const {
  std::unique_ptr<Scene> scene;
  from_json(content, scene);
  return make_problem_description(seed, *scene);
}

void NavigationProblemConverter::toJson_(
    nlohmann::json &recipient, const CartPosesConnector &connector) const {
  to_json(recipient, *connector.scene);
}

void NavigationProblemConverter::toJson(nlohmann::json &recipient,
                                        const std::vector<State> &sol,
                                        const Connector &connector) const {
  if (dynamic_cast<const CartPosesConnector *>(&connector) == nullptr) {
    throw Error{"not a valid connector"};
  }

  auto throw_exc = []() {
    throw Error{"something went wrong logging solution found"};
  };

  auto prev = sol.begin();
  auto current = sol.begin() + 1;
  std::vector<mt_rrt::State> interpolated;
  for (; current != sol.end(); ++current, ++prev) {
    interpolated.push_back(*prev);
    auto traj = connector.getTrajectory(*prev, *current);
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
      recipient.push_back(traj->getState());
    }
  }
  recipient.push_back(sol.back());
  recipient = interpolated;
}
} // namespace mt_rrt::samples
