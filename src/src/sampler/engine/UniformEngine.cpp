/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <sampler/engine/UniformEngine.h>

namespace mt::sampling {
    UniformEngine::UniformEngine(const float& lowerBound, const float& upperBound)
        : RandomEngine<std::uniform_real_distribution<float>>(lowerBound, upperBound) {
    }

    UniformEngine::UniformEngine()
        : UniformEngine(0.f, 1.f) {
    }
}
