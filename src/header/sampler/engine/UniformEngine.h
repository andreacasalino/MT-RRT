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
    class UniformEngine : public RandomEngine<std::uniform_real_distribution<float>> {
    public:
        UniformEngine(const float& lowerBound, const float& upperBound);
        UniformEngine();  // [0,1]
    };
}

#endif