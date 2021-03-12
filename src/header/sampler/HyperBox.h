/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_HYPER_BOX_H
#define MT_RRT_HYPER_BOX_H

#include <sampler/Sampler.h>
#include <sampler/engine/UniformEngine.h>

namespace mt::sampling {
	class HyperBox : public Sampler {
    public:
        HyperBox(const NodeState lowerCorner, const NodeState upperCorner);

        std::unique_ptr<Sampler> copy() const override;

        NodeState randomState() const override;

        inline const NodeState& getLowerLimit() const { return this->lowerLimits; };
        inline const NodeState& getDeltaLimit() const { return this->deltaLimits; };

    private:
    // data
        UniformEngine engine;
        const NodeState lowerLimits;
        const NodeState deltaLimits;
	};
}

#endif