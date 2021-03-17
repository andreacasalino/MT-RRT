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
#include <JSONstream.h>

namespace mt::sample {
    struct ProblemData {
        std::vector<Manipulator> robots;
        std::vector<geometry::Sphere> obstacles;
    };

    class ManipulatorProblem : public Problem {
    public:
        ManipulatorProblem(const std::vector<Manipulator>& robots, const std::vector<geometry::Sphere>& obstacles);

        const std::vector<Manipulator>& getRobots() const;

        const std::vector<geometry::Sphere>& getObstacles() const;

        structJSON getJSON() const;
    };

    std::tuple<ProblemPtr, NodeState, NodeState> importProblem(const std::string& configFileName);

    NodeState degree2rad(const NodeState& pose);
}

#endif