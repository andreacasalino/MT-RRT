/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_POINT2_H
#define MT_RRT_SAMPLE_POINT2_H

#include <Box.h>
#include <Problem.h>
#include <Logger.h>

namespace mt::sample {
    class Point2D : public Problem {
    public:
        Point2D(const sample::Box& boundaries, const std::vector<sample::Box>& obstacles);

        inline std::unique_ptr<Problem> copy() const override { return std::make_unique<Point2D>(this->getBoundaries(), this->getObstacles()); };

        std::vector<sample::Box> getObstacles() const;

        sample::Box getBoundaries() const;

        void log(Logger& log) const;
    };
}

#endif