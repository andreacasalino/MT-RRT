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
        std::vector<Sphere> obstacles;
    };

    class ManipulatorProblem : public Problem {
    public:
        ManipulatorProblem(const std::vector<Manipulator>& robots, const std::vector<Sphere>& obstacles);

        inline std::unique_ptr<Problem> copy() const override { return std::make_unique<ManipulatorProblem>(this->getRobots(), this->getObstacles()); };

        inline const std::vector<Manipulator>& getRobots() const { return this->data->robots; };

        inline const std::vector<Sphere>& getObstacles() const { return this->data->obstacles; };

        structJSON getJSON() const;

    private:
        std::shared_ptr<ProblemData> data;
    };

    std::tuple<ProblemPtr, NodeState, NodeState> importProblem(const std::string& configFileName);
}

#endif