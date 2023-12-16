/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <LogResult.h>
#include <TrivialProblem.h>

namespace mt_rrt {
void to_json(LogResult &j, const trivial::TrivialProblemConnector &subject);
} // namespace mt_rrt
