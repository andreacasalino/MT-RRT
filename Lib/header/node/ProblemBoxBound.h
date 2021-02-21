/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_PROBLEM_BOX_H
#define MT_RRT_PROBLEM_BOX_H

#include <node/ProblemDecorator.h>
#include <UniformSampler.h>

namespace mt::node {
	class ProblemBox : public ProblemDecorator {
    public:
        NodeState randomState() override;

	protected:
        ProblemBox(std::unique_ptr<Problem> wrapped, const NodeState lowerCorner, const NodeState upperCorner);

    private:
    // data
        const NodeState lowerLimits;
        const NodeState deltaLimits;
        UniformSampler engine;
	};
}

#endif