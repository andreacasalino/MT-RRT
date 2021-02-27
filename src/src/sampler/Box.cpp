/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <sampler/Box.h>
#include <Error.h>
#include <time.h>
#include <random>

namespace mt::sampling {
    NodeState computeDelta(const NodeState lowerCorner, const NodeState upperCorner) {
        NodeState delta = upperCorner;
        for (std::size_t k = 0; k < lowerCorner.size(); ++k) {
            if (lowerCorner[k] > upperCorner[k]) {
                throw Error("found lower limit greater than corresponding upper one");
            }
            delta[k] -= lowerCorner[k];
        }
        return delta;
    }
    Box::Box(const NodeState lowerCorner, const NodeState upperCorner, const unsigned int& seed)
        : lowerLimits(lowerCorner) 
        , deltaLimits(computeDelta(lowerCorner, upperCorner))
        , seed(seed) {
        if (0 == seed) {
            this->seed = static_cast<unsigned int>(time(NULL));
        }
    }

    std::unique_ptr<Sampler> Box::copy() const {
        return std::make_unique<Box>(*this);
    }

    NodeState Box::randomState() const {
        NodeState randState = this->lowerLimits;
        for (std::size_t k = 0; k < this->deltaLimits.size(); ++k) {
            randState[k] += this->deltaLimits[k] * this->engine();
        }
        return randState;
    }
}