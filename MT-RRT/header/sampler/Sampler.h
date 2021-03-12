/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLER_H
#define MT_RRT_SAMPLER_H

#include <Copiable.h>
#include <Node.h>

namespace mt::sampling {
	class Sampler : public Copiable<Sampler> {
	public:
		/** \brief Returns a node having a state randomly sampled in the \mathcal{X} space, Section 1.2.1 of the documentation.
		\details This function is invoked for randomly growing a searching tree.
		* @param[out] return the random node computed . 
		*/
        virtual NodeState randomState() const = 0;
	};

    typedef std::unique_ptr<Sampler> SamplerPtr;
}

#endif