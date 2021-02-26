/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_SAMPLER_H
#define MT_RRT_SAMPLER_H

#include <Node.h>
#include <random>

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

    class UniformRandomEngine {
    public:
        UniformRandomEngine(const float& l, const float& U);
        UniformRandomEngine(); //[0,1]

        inline float operator()() const { return this->distribution(this->generator); };

    private:
        mutable std::default_random_engine generator;
        mutable std::uniform_real_distribution<float> distribution;
    };
}

#endif