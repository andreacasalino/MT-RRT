/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_UNIFORM_ENGINE_H
#define MT_RRT_UNIFORM_ENGINE_H

#include <random>

namespace mt::sampling {
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