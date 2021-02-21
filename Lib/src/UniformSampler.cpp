/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <UniformSampler.h>
#include <time.h>
#include <stdlib.h>

namespace mt {
    UniformSampler::UniformSampler() {
        this->seed = static_cast<unsigned int>(time(NULL));
    }

    UniformSampler::UniformSampler(const unsigned int& s) {
        this->seed = s;
    }

    float UniformSampler::sample(const float& min, const float& delta) {
        return min + delta * static_cast<float>(std::rand_r(&this->seed)) / static_cast<float>(RAND_MAX);
    }
}