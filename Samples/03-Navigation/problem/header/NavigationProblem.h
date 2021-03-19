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

namespace mt::sample {
    struct Description {
        geometry::Rectangle boundaries;
        std::vector<geometry::Sphere> obstacles;
        Cart cart;
        float blendRadius;
    };

    // std::tuple<ProblemPtr, NodeState, NodeState> importNavigationProblem(const std::string& configFileName);
}

#endif