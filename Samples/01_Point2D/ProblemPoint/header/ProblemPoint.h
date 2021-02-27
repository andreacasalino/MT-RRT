/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_PROBLEM_POINT_H
#define MT_RRT_SAMPLE_PROBLEM_POINT_H

#include <ProblemEuclidean.h>
#include <Box.h>

namespace mt::sample {
    class ProblemPoint : public ProblemEuclidean {
    public:
        ProblemPoint(const Box& boundaries, const std::vector<Box>& obstacles);

        std::unique_ptr<Problem> copy() const override;

        const std::vector<Box>& getObstacles() const;

        const Box& getBoundaries() const;
    };
}

#endif