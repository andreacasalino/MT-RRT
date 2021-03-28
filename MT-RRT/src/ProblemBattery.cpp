/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include "ProblemBattery.h"
#include <Error.h>

namespace mt {
    void checkBattery(const std::vector<ProblemPtr>& problems) {
        if(problems.size() <= 1){
            throw Error("not enough problem provided for a multi threading strategy");
        }
    }

    ProblemBattery::ProblemBattery(const std::vector<ProblemPtr>& battery) {
        checkBattery(battery);
        this->problems.reserve(battery.size());
        for (auto it = battery.begin(); it != battery.end(); ++it) {
            this->problems.push_back(it->get());
        }
    }
}