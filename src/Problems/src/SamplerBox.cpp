/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <SamplerBox.h>
#include <Error.h>
#include <time.h>
#include <random>

namespace mt {
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
    SamplerBox::SamplerBox(const NodeState lowerCorner, const NodeState upperCorner, const unsigned int& seed)
        : lowerLimits(lowerCorner) 
        , deltaLimits(computeDelta(lowerCorner, upperCorner))
        , seed(seed) {
        if (0 == seed) {
            this->seed = static_cast<unsigned int>(time(NULL));
        }
    }

    std::unique_ptr<Sampler> SamplerBox::copy() const {
        return std::make_unique<SamplerBox>(*this);
    }

    NodeState SamplerBox::randomState() {
        NodeState randState = this->lowerLimits;
        throw Error("replace rand with something thread safe");
        for (std::size_t k = 0; k < this->deltaLimits.size(); ++k) {
            randState[k] += this->deltaLimits[k] * static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
        }
        return randState;
    }
}