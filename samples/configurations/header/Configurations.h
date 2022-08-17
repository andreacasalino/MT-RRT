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

namespace mt_rrt::samples {
static const std::string HELP = R"(
-h, --help                  --> shows this help again\n\n
-f FILE                     --> reads config.json from file at FILE. See -j for the expected json format\n\n
-j "CONFIG_JSON_AS_STRING"  --> json describing settings to apply. See mt_rrt::samples::from_json(const nlohmann::json &, Parameters&) and mt_rrt::samples::from_json(const nlohmann::json &, ProblemDescription &&problem) for the expected json format
)";

// Converts information passed in argc and argv into a json that is used to set
// up some mt_rrt::Parameters and then build a mt_rrt::Planner to use
// See HELP
void to_json(nlohmann::json &configurations, int argc, const char **argv);

// Overrides configurations in passed recipient, if something in the
// passed json is found.
//
// All fieds that can be overridden, are assumed to be optional, i.e.
// may be or not defined in the passed json.
// Recognized fields inside j["params"] are:
// j["params"]["strategy"] --> a string that can be equal to: "Single", "Bidir"
// or "Star" j["params"]["steer_trials"] --> positive number
// j["params"]["iterations"] --> number
// j["params"]["determinism"] --> floating number in [0,1]
// j["params"]["best_effort] --> boolean
void from_json(const nlohmann::json &j, Parameters &recipient);

// Instantiate and setup a planner according to the configurations contained in
// the json. Recognized fields inside j["planner"] are:
// j["planner"]["type"] --> a string that can be equal to:
//  - "embarassingly"
//  - "parall_query"
//  - "shared"
//  - "linked"
//  - "agents"
// if this field is not specified a StandardPlanner is returned.
// On the contrary case, the multi threaded planner specified in
// j["planner"]["type"] is created.
//
// In this last case, some additional fields
// will be considered to set up the returned planner:
//
// j["planner"]["threads"] --> the number of threads
//
// j["planner"]["synch"] --> number in [0,1]to specify the synchronization
// degree. Actually, this field is considered only in case j["planner"]["type"]
// was set equal to "linked" or "agents"
std::unique_ptr<Planner> from_json(const nlohmann::json &j,
                                   ProblemDescription &&problem);
} // namespace mt_rrt::samples
