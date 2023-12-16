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
namespace {
struct SeedGenerationContext {
  std::mutex mtx;
  std::chrono::steady_clock::time_point program_begin =
      std::chrono::steady_clock::now();
};
static SeedGenerationContext info;
} // namespace

Seed make_random_seed() {
  std::scoped_lock lock{info.mtx};
  auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - info.program_begin);
  Seed result = static_cast<Seed>(elapsed.count());
  if (result == detail::MAX_POSSIBLE_SEED) {
    info.program_begin = std::chrono::steady_clock::now();
  }
  return result;
}

GaussianEngine::GaussianEngine(const float &mean, const float &stdDeviation,
                               const std::optional<Seed> &seed)
    : detail::Distribution<std::normal_distribution<float>>(
          std::normal_distribution<float>{mean, stdDeviation}, seed) {
  // TODO validate mean and std_dev?
}

UniformEngine::UniformEngine(const float &lowerBound, const float &upperBound,
                             const std::optional<Seed> &seed)
    : detail::Distribution<std::uniform_real_distribution<float>>(
          std::uniform_real_distribution<float>{lowerBound, upperBound}, seed) {
  // TODO validate lower upper bounds?
}
} // namespace mt_rrt
