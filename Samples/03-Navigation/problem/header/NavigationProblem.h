/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_NAVIGATION_PROBLEM_H
#define MT_RRT_SAMPLE_NAVIGATION_PROBLEM_H

#include <Rectangle.h>
#include <Sphere.h>
#include <Problem.h>
#include <JSONstream.h>

namespace mt::sample {
    class NavigationProblem : public Problem {
    public:
        NavigationProblem(const geometry::Rectangle& boundaries, const std::vector<geometry::Sphere>& obstacles);

        const std::vector<geometry::Sphere>& getObstacles() const;

        geometry::Rectangle getBoundaries() const;

        structJSON getJSON() const;
    };
}

#endif