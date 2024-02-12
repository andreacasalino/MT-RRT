/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <LogResult.h>
#include <NavigationProblem.h>

namespace mt_rrt {
void to_json(nlohmann::json &j, const navigation::Cart &subject);

void to_json(LogResult &j, const navigation::CartPosesConnector &subject);

// interpolate the solution to add more states for the purpose of visualize the
// trajectory
std::vector<std::vector<float>>
interpolate(const navigation::CartPosesConnector &connector,
            const std::vector<std::vector<float>> &solution);
} // namespace mt_rrt
