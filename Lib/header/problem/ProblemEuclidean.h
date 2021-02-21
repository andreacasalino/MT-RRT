/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_PROBLEM_EUCLIDEAN_H
#define MT_RRT_PROBLEM_EUCLIDEAN_H

#include <problem/Problem.h>

namespace mt::problem {
    class ProblemEuclidean : public Problem {
    public:
        float cost2Go(const Node& start, const Node& ending_node, const bool& ignoreConstraints) override;

        TrajectoryPtr getTrajectory(const Node& start, const Node& trg) override;

    protected:
        ProblemEuclidean(SamplerPtr sampler, CheckerPtr checker, const std::size_t& stateSpaceSize, const float& gamma, const float& steerDegree);

    private:
        float steerDegree;
    };
}

#endif