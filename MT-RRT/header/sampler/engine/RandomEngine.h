/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_RANDOM_ENGINE_H
#define MT_RRT_RANDOM_ENGINE_H

#include <sampler/engine/SeedFactory.h>

namespace mt::sampling {
    /** @brief An object used to drawn samples in a thread safe manner.
     * Use one of the ancestors of this interface instead of rand(), which is not thread safe
     */
    template<typename Distribution>
    class RandomEngine {
    public:
        /** @brief Draw an return a new sample
         */
        inline float operator()() const { return this->distribution(this->generator); };

    protected:
        template<typename ... Args>
        RandomEngine(Args ... args) 
            : distribution(args...) {
            SeedFactory::addToRegister(&this->generator);
        };
        RandomEngine(const RandomEngine& o)
            : generator(o.generator)
            , distribution(o.distribution) {
            SeedFactory::addToRegister(&this->generator);
        };
        ~RandomEngine() {
            SeedFactory::removeFromRegister(&this->generator);
        };

        mutable std::default_random_engine generator;
        mutable Distribution distribution;
    };
}

#endif