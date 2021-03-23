/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_GAUSSIAN_ENGINE_H
#define MT_RRT_GAUSSIAN_ENGINE_H

#include <sampler/engine/RandomEngine.h>

namespace mt::sampling {
    /** @brief Used to draw samples from a normal distribution
     */
    class GaussianEngine : public RandomEngine<std::normal_distribution<float>> {
    public:
        /** @param the mean of the normal distribution
         *  @param the standard deviation of the normal distribution
         */
        GaussianEngine(const float& mean, const float& stdDeviation);

        /** @brief Similar to GaussianEngine::GaussianEngine(const float& mean, const float& stdDeviation),
         * assuming 0 as mean and 1 as standard deviation
         */
        GaussianEngine();
    };
}

#endif