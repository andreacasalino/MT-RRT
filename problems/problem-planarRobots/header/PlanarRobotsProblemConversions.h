/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <LogResult.h>
#include <PlanarRobotsProblem.h>

namespace mt_rrt {
void to_json(nlohmann::json &j, const robots::Robot &subject);

void to_json(LogResult &j, const robots::PosesConnector &subject);
} // namespace mt_rrt
