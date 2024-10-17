/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <LogResult.h>
#include <MiscConversions.h>

namespace mt_rrt {
LogResult::LogResult() {
  obstacles = nlohmann::json::array();
  trees = nlohmann::json::array();
  solutions = nlohmann::json::array();
}

void LogResult::addTree(const PlannerSolution::TreeSerialized &tree) {
  to_json(trees.emplace_back(), tree);
}

void LogResult::addSolution(const std::vector<std::vector<float>> &sequence, float cost) {
  auto &added = solutions.emplace_back();
  added["cost"] = cost;
  added["sequence"] = sequence;
}
} // namespace mt_rrt
