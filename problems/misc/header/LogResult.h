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

#include <Logger.h>
#include <MiscConversions.h>

namespace mt_rrt {
template <typename ObstacleT> struct ObstacleTrait {};

template <> struct ObstacleTrait<geom::Box> {
  static std::string typeStr() { return "Box"; }
};

template <> struct ObstacleTrait<geom::Segment> {
  static std::string typeStr() { return "Segment"; }
};

template <> struct ObstacleTrait<geom::Sphere> {
  static std::string typeStr() { return "Sphere"; }
};

class LogResult {
public:
  LogResult();

  const auto &get() const { return content; }

  nlohmann::json &addToScene(const std::string &key) { return scene[key]; }

  template <typename ObstacleT> void addObstacle(const ObstacleT &subject) {
    auto &added = obstacles.emplace_back();
    to_json(added, subject);
    added["type"] = ObstacleTrait<ObstacleT>::typeStr();
  }

  void addTree(const TreeHandler &tree);

  void addSolution(const std::vector<std::vector<float>> &sequence);
  void addSolution(const Solution &solution);

  void addPlannerSolution(const PlannerSolution &sol) {
    for (const auto &tree : sol.trees) {
      addTree(*tree);
    }
    auto &added = solutions.emplace_back();
    added["sequence"] = sol.solution;
  }

private:
  nlohmann::json content;
  nlohmann::json &scene = content["scene"];
  nlohmann::json &obstacles = scene["obstacles"];
  nlohmann::json &trees = content["trees"];
  nlohmann::json &solutions = content["solutions"];
};
} // namespace mt_rrt
