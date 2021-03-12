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
    class GaussianEngine : public RandomEngine<std::normal_distribution<float>> {
    public:
        GaussianEngine(const float& mean, const float& stdDeviation);
        GaussianEngine();  // [0,1]
    };
}

#endif