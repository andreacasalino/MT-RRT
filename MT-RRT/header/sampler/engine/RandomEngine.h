/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_RANDOM_ENGINE_H
#define MT_RRT_RANDOM_ENGINE_H

#include <random>

namespace mt::sampling {
    template<typename Distribution>
    class RandomEngine {
    public:
        inline float operator()() const { return this->distribution(this->generator); };

    protected:
        template<typename ... Args>
        RandomEngine(Args ... args) 
            : distribution(args...) {
        };

        mutable std::default_random_engine generator;
        mutable Distribution distribution;
    };
}

#endif