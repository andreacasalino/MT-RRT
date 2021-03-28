/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_MANIPULATOR_PROBLEM_H
#define MT_RRT_SAMPLE_MANIPULATOR_PROBLEM_H

#include <Manipulator.h>
#include <Sphere.h>
#include <Problem.h>

namespace mt::sample {
    /** @brief The Problem described in Section 2.2 of the documentation
     */
    struct Description {
        std::vector<Manipulator> robots;
        std::vector<geometry::Sphere> obstacles;
    };

    NodeState degree2rad(const NodeState& pose);

    /** @brief Used to select between the two advance approaches described in Sections 2.2.3.1 (Tunneled) and Sections 2.2.3.2 (BubbleFreeConfiguration) of the documentation
     */
    enum AdvanceApproach { Tunneled, BubbleFreeConfiguration };

    std::tuple<ProblemPtr, NodeState, NodeState> importManipulatorProblem(const std::string& configFileName, const AdvanceApproach& advanceAppr);

    ProblemPtr importInterpolator(const std::string& configFileName);
}

#endif