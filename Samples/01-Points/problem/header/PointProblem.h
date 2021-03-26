/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_POINT_PROBLEM_H
#define MT_RRT_SAMPLE_POINT_PROBLEM_H

#include <Rectangle.h>
#include <Problem.h>

namespace mt::sample {
    /** @brief The Problem described in Section 2.1 of the documentation
     */
    struct Description {
        geometry::Rectangle boundaries;
        std::vector<geometry::Rectangle> obstacles;
    };

    ProblemPtr makeProblemPoint(const geometry::Rectangle& boundaries, const std::vector<geometry::Rectangle>& obstacles);
}

#endif