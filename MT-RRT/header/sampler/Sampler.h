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
	/** @brief Interface for a sampler of states.
	 */
	class Sampler : public Copiable<Sampler> {
	public:
		/** @brief Returns a node having a state randomly sampled in the \mathcal{X} space, Section METTERE of the documentation.
		 * This function is invoked mainly for randomly growing a searching tree.
		 * @return a drawn random state. 
		 */
        virtual NodeState randomState() const = 0;
	};

    typedef std::unique_ptr<Sampler> SamplerPtr;
}

#endif