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
    /** @brief A sampler drawing a sample inside an hypercube of n-dimensions, described
     * by 2 corners.
     * For example, corners [l1, l2, l3, l4] and [u1, u2, u3, u4], describe an hyperbox
     * whose points [x1,x2,x3,x4] are all such that:
     * li <= xi <= ui  
     */
	class HyperBox : public Sampler {
    public:
        /** @param the lower corner of the hyperbox 
         *  @param the upper corner of the hyperbox
         */
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