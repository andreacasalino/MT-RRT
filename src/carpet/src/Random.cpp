/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT-carpet/Random.h>

#include <chrono>
#include <mutex>

namespace mt_rrt {
namespace {
class SeedRandomGenerator {
public:
  static Seed makeSeed() {
    std::scoped_lock lock(program_begin_mtx);
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - program_begin);
    Seed result = elapsed.count();
    if (result == detail::MAX_POSSIBLE_SEED) {
      program_begin = std::chrono::steady_clock::now();
    }
    return result;
  }

private:
  static std::mutex program_begin_mtx;
  static std::chrono::steady_clock::time_point program_begin;
};

std::mutex SeedRandomGenerator::program_begin_mtx = std::mutex{};
std::chrono::steady_clock::time_point SeedRandomGenerator::program_begin =
    std::chrono::steady_clock::now();
} // namespace

Seed make_random_seed() { return SeedRandomGenerator::makeSeed(); }

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
