/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Random.h>

#include <chrono>
#include <mutex>

namespace mt_rrt {
Seed make_random_seed() {
  std::uint64_t maxSeed{detail::MAX_POSSIBLE_SEED};
  std::uint64_t now = std::chrono::high_resolution_clock::now().time_since_epoch().count() % maxSeed;
  return static_cast<Seed>(now);
}

GaussianEngine::GaussianEngine(float mean, float stdDeviation,
                               const std::optional<Seed> &seed)
    : detail::Distribution<std::normal_distribution<float>>(
          std::normal_distribution<float>{mean, stdDeviation}, seed) {
  // TODO validate mean and std_dev?
}

UniformEngine::UniformEngine(float lowerBound, float upperBound,
                             const std::optional<Seed> &seed)
    : detail::Distribution<std::uniform_real_distribution<float>>(
          std::uniform_real_distribution<float>{lowerBound, upperBound}, seed) {
  // TODO validate lower upper bounds?
}
} // namespace mt_rrt
