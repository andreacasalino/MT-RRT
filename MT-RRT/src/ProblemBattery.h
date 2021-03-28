/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_PROBLEM_BATTERY_H
#define MT_RRT_PROBLEM_BATTERY_H

#include <Problem.h>

namespace mt {
    void checkBattery(const std::vector<ProblemPtr>& problems);

    class ProblemBattery {
    public:
        ProblemBattery(const std::vector<ProblemPtr>& problems);

    protected:
        std::vector<Problem*> problems;
    };
}

#endif