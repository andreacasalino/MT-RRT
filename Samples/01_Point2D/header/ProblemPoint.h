/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLE_PROBLEM_POINT_H
#define MT_RRT_SAMPLE_PROBLEM_POINT_H

#include <ProblemEuclidean.h>

namespace mt::sample {
    class ProblemPoint : public ProblemEuclidean {
    public:


        std::unique_ptr<Problem> copy() const override;

    private:

    };
}

#endif