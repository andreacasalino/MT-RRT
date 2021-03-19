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
#include <JSONstream.h>

namespace mt::sample {
    class PointProblem : public Problem {
    public:
        PointProblem(const geometry::Rectangle& boundaries, const std::vector<geometry::Rectangle>& obstacles);

        const std::vector<geometry::Rectangle>& getObstacles() const;

        geometry::Rectangle getBoundaries() const;

        structJSON getJSON() const;
    };
}

#endif