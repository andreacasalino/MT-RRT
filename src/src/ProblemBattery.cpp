/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "Commons.h"

namespace mt {
    ProblemBattery::ProblemBattery(const std::vector<ProblemPtr>& problemcopies)
        : MtObject(problemcopies.size()) {
        this->problems.reserve(problemcopies.size());
        for (auto it = problemcopies.begin(); it != problemcopies.end(); ++it) {
            this->problems.emplace_back(it->get());
        }
    }
}