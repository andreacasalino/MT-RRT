/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_POINT_PROBLEM_H
#define MT_RRT_SAMPLE_POINT_PROBLEM_H

#include <Obstacle.h>
#include <Problem.h>
#include <JSONstream.h>

namespace mt::sample {
    class PointProblem : public Problem {
    public:
        PointProblem(const sample::Obstacle& boundaries, const std::vector<sample::Obstacle>& obstacles);

        inline std::unique_ptr<Problem> copy() const override { return std::make_unique<PointProblem>(this->getBoundaries(), this->getObstacles()); };

        const std::vector<sample::Obstacle>& getObstacles() const;

        sample::Obstacle getBoundaries() const;

        structJSON getJSON() const;
    };
}

#endif