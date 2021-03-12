/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <sampler/HyperBox.h>
#include <Error.h>
#include <time.h>
#include <random>

namespace mt::sampling {
    NodeState computeDelta(const NodeState lowerCorner, const NodeState upperCorner) {
        if(lowerCorner.size() != upperCorner.size()) {
            throw Error("lower and upper corners should have the same sizes");
        }
        NodeState delta = upperCorner;
        for (std::size_t k = 0; k < lowerCorner.size(); ++k) {
            if (lowerCorner[k] > upperCorner[k]) {
                throw Error("found lower limit greater than corresponding upper one");
            }
            delta[k] -= lowerCorner[k];
        }
        return delta;
    }
    HyperBox::HyperBox(const NodeState lowerCorner, const NodeState upperCorner)
        : lowerLimits(lowerCorner) 
        , deltaLimits(computeDelta(lowerCorner, upperCorner)) {
    }

    std::unique_ptr<Sampler> HyperBox::copy() const {
        return std::make_unique<HyperBox>(*this);
    }

    NodeState HyperBox::randomState() const {
        NodeState randState = this->lowerLimits;
        for (std::size_t k = 0; k < this->deltaLimits.size(); ++k) {
            randState[k] += this->deltaLimits[k] * this->engine();
        }
        return randState;
    }
}