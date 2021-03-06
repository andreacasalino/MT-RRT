/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_MANIPULATOR_PROBLEM_H
#define MT_RRT_SAMPLE_MANIPULATOR_PROBLEM_H

#include <Manipulator.h>
#include <Problem.h>
#include <JSONstream.h>

namespace mt::sample {
    class Sphere {
    public:
        Sphere(const float& x, const float& y, const float& ray);

        Sphere(const Sphere&) = default;
        Sphere& operator=(const Sphere&) = default;

        inline const geometry::Point& getCenter() const { return this->center; };
        inline const float& getRay() const { return this->ray; };

    private:
        geometry::Point center;
        float ray;
    };

    struct ProblemData {
        std::vector<Manipulator> robots;
        std::vector<Sphere> obstacles;
    };

    class ManipulatorProblem : public Problem {
    public:
        ManipulatorProblem(const std::vector<Manipulator>& robots, const std::vector<Sphere>& obstacles);
        //ManipulatorProblem(const std::string& jsonFile);

        inline std::unique_ptr<Problem> copy() const override { return std::make_unique<ManipulatorProblem>(this->getRobots(), this->getObstacles()); };

        inline const std::vector<Manipulator>& getRobots() const { return this->data->robots; };

        inline const std::vector<Sphere>& getObstacles() const { return this->data->obstacles; };

        //structJSON getJSON() const;

    private:
        std::shared_ptr<ProblemData> data;
    };
}

#endif