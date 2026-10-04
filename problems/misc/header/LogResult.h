/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Extender.h>
#include <MT-RRT/Node.h>
#include <MT-RRT/Planner.h>

#include <JsonConversions.h>
#include <Logger.h>

#include <memory>
#include <string_view>

namespace mt_rrt {
template <typename O>
concept RecognizedObstacle = requires(const O obj, nlohmann::json &recipient) {
  { to_json(recipient, obj) } -> std::same_as<void>;

  { O::type_name } -> std::same_as<std::string_view>;
};

class LogResult {
public:
  LogResult();

  LogResult(LogResult &&) = default;
  LogResult &operator=(LogResult &&) = default;

  const auto &get() const { return content; }

  nlohmann::json &addToScene(const std::string &key) { return scene[key]; }

  template <RecognizedObstacle O> void addObstacle(const O &subject) {
    auto &added = obstacles.emplace_back();
    to_json(added, subject);
    added["type"] = O::type_name;
  }

  void addTree(const Nodes &tree) { to_json(trees.emplace_back(), tree); }

  void addSolution(const Solution &solution);

  void addPlannerSolution(const PlannerSolution &sol);

private:
  std::unique_ptr<nlohmann::json> content;
  nlohmann::json &scene = (*content)["scene"];
  nlohmann::json &obstacles = scene["obstacles"];
  nlohmann::json &trees = (*content)["trees"];
  nlohmann::json &solutions = (*content)["solutions"];
};
} // namespace mt_rrt
