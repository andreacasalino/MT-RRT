/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <LogResult.h>

namespace mt_rrt {
LogResult::LogResult()
    : content{std::make_unique<nlohmann::json>()}, scene{(*content)["scene"]},
      obstacles{scene["obstacles"]}, trees{(*content)["trees"]},
      solutions{(*content)["solutions"]} {
  obstacles = nlohmann::json::array();
  trees = nlohmann::json::array();
  solutions = nlohmann::json::array();
}

void LogResult::addSolution(const Solution &solution) {
  auto &added = solutions.emplace_back();
  added["cost"] = solution.cost();
  auto &sequence = added["sequence"];
  sequence = nlohmann::json::array();
  auto it = solution.iter();
  while (true) {
    if (auto next = it.next(); next.has_value()) {
      sequence.emplace_back() = *next;
    } else {
      break;
    }
  }
}
} // namespace mt_rrt
