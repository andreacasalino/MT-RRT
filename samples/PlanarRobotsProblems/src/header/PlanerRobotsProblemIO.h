/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <JsonConvert.h>
#include <PlanarRobotsProblem.h>

namespace mt_rrt::samples {
void to_json(nlohmann::json &j, const Sphere &subject);
void to_json(nlohmann::json &j, const Robot &subject);
void to_json(nlohmann::json &j, const Scene &subject);

void from_json(const nlohmann::json &j, Sphere &subject);
void from_json(const nlohmann::json &j, Robot &subject);
void from_json(const nlohmann::json &j, Scene &subject);

class PosesConnectorLogger
    : public utils::ConnectorLoggerTyped<PosesConnector> {
public:
  static const PosesConnectorLogger LOGGER;

protected:
  void log(nlohmann::json &j, const PosesConnector &c) const final {
    j = *c.scene;
  };
};

std::shared_ptr<ProblemDescription>
make_problem_description(const std::optional<Seed> &seed,
                         const std::string &scene_json_filename, State &start,
                         State &end);
} // namespace mt_rrt::samples
