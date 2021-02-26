/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLER_H
#define MT_RRT_SAMPLER_H

#include <Node.h>

namespace mt {
	class Sampler {
	public:
		virtual	~Sampler() = default;

        virtual std::unique_ptr<Sampler> copy() const = 0;

        virtual NodeState randomState() = 0;

    protected:
        Sampler() = default;
	};

    typedef std::unique_ptr<Sampler> SamplerPtr;
}

#endif