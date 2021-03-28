/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_UNIFORM_ENGINE_H
#define MT_RRT_UNIFORM_ENGINE_H

#include <sampler/engine/RandomEngine.h>

namespace mt::sampling {
    /** @brief Used to draw sample whithin a compact inverval [l, U]
     */
    class UniformEngine : public RandomEngine<std::uniform_real_distribution<float>> {
    public:
        /** @param the lower bound of the compact interval
         *  @param the upper bound of the compact interval
         */
        UniformEngine(const float& lowerBound, const float& upperBound);

        /** @brief Similar to UniformEngine::UniformEngine(const float& lowerBound, const float& upperBound),
         * assuming 0 as lower bound and 1 as the upper one
         */
        UniformEngine();
    };
}

#endif