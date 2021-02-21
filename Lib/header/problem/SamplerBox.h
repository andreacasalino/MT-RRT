/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLER_BOX_H
#define MT_RRT_SAMPLER_BOX_H

#include <problem/Sampler.h>

namespace mt::problem {
	class SamplerBox : public Sampler {
    public:
        SamplerBox(const NodeState lowerCorner, const NodeState upperCorner, const unsigned int& seed = 0);

        std::unique_ptr<Sampler> copy() const override;

        NodeState randomState() override;

    private:
    // data
        const NodeState lowerLimits;
        const NodeState deltaLimits;
        unsigned int seed;
	};
}

#endif