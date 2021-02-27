/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLER_BOX_H
#define MT_RRT_SAMPLER_BOX_H

#include <sampler/Sampler.h>
#include <sampler/UniformEngine.h>

namespace mt::sampling {
	class Box : public Sampler {
    public:
        Box(const NodeState lowerCorner, const NodeState upperCorner, const unsigned int& seed = 0);

        std::unique_ptr<Sampler> copy() const override;

        NodeState randomState() const override;

        inline const NodeState& getLowerLimit() const { return this->lowerLimits; };
        inline const NodeState& getDeltaLimit() const { return this->deltaLimits; };

    private:
    // data
        UniformRandomEngine engine;
        const NodeState lowerLimits;
        const NodeState deltaLimits;
        unsigned int seed;
	};
}

#endif