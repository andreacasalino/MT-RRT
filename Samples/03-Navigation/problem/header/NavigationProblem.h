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
#include <Cart.h>
#include <Problem.h>
#include <JSONstream.h>

namespace mt::sample {
    struct ProblemData {
        geometry::Rectangle boundaries;
        std::vector<geometry::Sphere> obstacles;
        Cart cart;
        float blendRadius;
    };

    class NavigationProblem : public Problem {
    public:
        NavigationProblem(const geometry::Rectangle& boundaries, const std::vector<geometry::Sphere>& obstacles);

        geometry::Rectangle getBoundaries() const;

        const std::vector<geometry::Sphere>& getObstacles() const;

        float  getBlendRadius() const;

        structJSON getJSON() const;
    };

    std::tuple<ProblemPtr, NodeState, NodeState> importProblem(const std::string& configFileName);
}

#endif