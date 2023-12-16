/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MiscConversions.h>
#include <NavigationProblemConversions.h>

namespace mt_rrt {
void to_json(nlohmann::json &j, const navigation::Cart &subject) {
  j["width"] = subject.getWidth();
  j["length"] = subject.getLength();
}

void to_json(LogResult &j, const navigation::CartPosesConnector &subject) {
  to_json(j.addToScene("cart"), subject.scene->cart);
  for (const auto &sphere : subject.scene->obstacles) {
    j.addObstacle(sphere);
  }
}

std::vector<std::vector<float>>
interpolate(const navigation::CartPosesConnector &connector,
            const std::vector<std::vector<float>> &solution) {
  auto throw_exc = []() {
    throw Error{"something went wrong logging solution found"};
  };

  auto prev = solution.begin();
  auto current = solution.begin() + 1;
  std::vector<std::vector<float>> interpolated;
  for (; current != solution.end(); ++current, ++prev) {
    interpolated.push_back(*prev);
    auto traj = connector.getTrajectory(*prev, *current);
    if (nullptr == traj) {
      throw_exc();
    }
    while (true) {
      auto status = traj->advance();
      if (status == Trajectory::AdvanceInfo::blocked) {
        throw_exc();
      }
      if (status == Trajectory::AdvanceInfo::targetReached) {
        break;
      }
      interpolated.emplace_back(traj->getState().convert());
    }
  }
  interpolated.push_back(solution.back());
  return interpolated;
}
} // namespace mt_rrt
