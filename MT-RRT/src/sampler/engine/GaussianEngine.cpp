/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <sampler/engine/GaussianEngine.h>

namespace mt::sampling {
    GaussianEngine::GaussianEngine(const float& mean, const float& stdDeviation)
        : RandomEngine<std::normal_distribution<float>>(mean, stdDeviation) {
    }

    GaussianEngine::GaussianEngine()
        : GaussianEngine(0.f, 1.f) {
    }
}
