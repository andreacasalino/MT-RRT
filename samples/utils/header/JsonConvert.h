/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/../../src/Extender.h>
#include <MT-RRT-core/Planner.h>

#include <nlohmann/json.hpp>

namespace mt_rrt::utils {
void from_file(nlohmann::json &j, const std::string &fileName);

void to_json(nlohmann::json &j, const Tree &subject);

void to_json(nlohmann::json &j, const std::vector<Tree> &subject);

void to_json(nlohmann::json &j, const ProblemDescription &problem,
             const PlannerSolution &solution, const Converter &converter);

void to_json(nlohmann::json &j, const Extender &subject,
             const Converter &converter);
} // namespace mt_rrt::utils
