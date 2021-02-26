/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <Sampler.h>

//https://www.cplusplus.com/reference/random/uniform_real_distribution/

namespace mt {
    UniformRandomEngine::UniformRandomEngine(const float& l, const float& U)
        : distribution(l, U) {
    }

    UniformRandomEngine::UniformRandomEngine()
        : distribution(0.f, 1.f) {
    }
}