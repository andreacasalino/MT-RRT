/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-core/Parameters.h>
#include <MT-RRT-core/Planner.h>

#include <nlohmann/json.hpp>

namespace mt_rrt::sample {
// TODO comment how argc and argv expected format
//
// TODO document the expect format for the config json
void to_json(nlohmann::json &configurations, int argc, const char **argv);

// TODO comment the fact that not specified field are ignored
void from_json(const nlohmann::json &j, Parameters &recipient);

// instantiate and setup a planner according to the configurations contained in
// the json.
//
// TODO METTERE trivial problem scenario 00 does inline something similar and
// you can have a look to that sample to see how to build and configure the
// planners inside this repo.
std::unique_ptr<Planner> from_json(const nlohmann::json &j,
                                   ProblemDescription &&problem);
} // namespace mt_rrt::sample
