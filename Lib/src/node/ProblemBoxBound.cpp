/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <node/ProblemBoxBound.h>
#include <Error.h>

namespace mt::node {
    NodeState computeDelta(const NodeState lowerCorner, const NodeState upperCorner, const std::size_t& size) {
        if (lowerCorner.size() != size) throw Error("lower limits don't match the problem size");
        if (upperCorner.size() != size) throw Error("upper limits don't match the problem size");
        NodeState delta = upperCorner;
        for (std::size_t k = 0; k < lowerCorner.size(); ++k) {
            if (lowerCorner[k] > upperCorner[k]) {
                throw Error("found lower limit greater than corresponding upper one");
            }
            delta[k] -= lowerCorner[k];
        }
        return delta;
    }
    ProblemBox::ProblemBox(std::unique_ptr<Problem> wrapped, const NodeState lowerCorner, const NodeState upperCorner)
        : ProblemDecorator(std::move(wrapped))
        , lowerLimits(lowerCorner) 
        , deltaLimits(computeDelta(lowerCorner, upperCorner, this->wrapped->getProblemSize())) {
    }

    NodeState ProblemBox::randomState() {
        NodeState randState = this->lowerLimits;
        for (std::size_t k = 0; k < this->deltaLimits.size(); ++k) {
            randState[k] += this->engine.sample(0.f, this->deltaLimits[k]);
        }
        return randState;
    }
}