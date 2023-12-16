/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <limits.h>
#include <optional>
#include <random>
#include <vector>

namespace mt_rrt {
using Seed = unsigned;

/**
 * @return time since start of the process is used to generated the seed
 */
Seed make_random_seed();

namespace detail {
static constexpr Seed MAX_POSSIBLE_SEED = std::numeric_limits<Seed>::max();

template <typename DistributionShape> class Distribution {
public:
  float sample() const { return distribution_shape(generator); };

  Distribution(const Distribution &o)
      : distribution_shape(o.distribution_shape) {
    generator.seed(o.sampleSeed());
  }
  Distribution &operator=(const Distribution &o) = delete;

  Distribution(Distribution &&o) = delete;
  Distribution &operator=(Distribution &&o) = delete;

  Seed sampleSeed() const {
    std::uniform_int_distribution<Seed> seed_distribution(0, MAX_POSSIBLE_SEED);
    return static_cast<Seed>(seed_distribution(generator));
  }

protected:
  Distribution(DistributionShape &&shape, const std::optional<Seed> &seed)
      : distribution_shape(std::move(shape)) {
    if (seed) {
      generator.seed(seed.value());
    } else {
      generator.seed(make_random_seed());
    }
  }

private:
  mutable std::default_random_engine generator;
  mutable DistributionShape distribution_shape;
};
} // namespace detail

/**
 * @brief Used to draw samples from a normal distribution
 */
class GaussianEngine
    : public detail::Distribution<std::normal_distribution<float>> {
public:
  /**
   * @param the mean of the normal distribution
   * @param the standard deviation of the normal distribution
   */
  GaussianEngine(const float &mean = 0, const float &stdDeviation = 1.f,
                 const std::optional<Seed> &seed = std::nullopt);
};

/**
 * @brief Used to draw sample whithin a compact inverval [l, U]
 */
class UniformEngine
    : public detail::Distribution<std::uniform_real_distribution<float>> {
public:
  /**
   * @param the lower bound of the compact interval
   * @param the upper bound of the compact interval
   */
  UniformEngine(const float &lowerBound = 0, const float &upperBound = 1.f,
                const std::optional<Seed> &seed = std::nullopt);
};
} // namespace mt_rrt
