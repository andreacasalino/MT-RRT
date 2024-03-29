/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Copiable.h>
#include <MT-RRT/Random.h>

#include <vector>

namespace mt_rrt {
/**
 * @brief Interface for a sampler of states.
 */
class Sampler : public Copiable<Sampler> {
public:
  virtual ~Sampler() = default;

  /**
   * @brief Returns a state randomly sampled in the \mathcal{X}
   * space, Sections 1.2.1, 1.2.2 and 1.2.3 of the documentation.
   * This random state are used for randomly growing searching trees.
   * @return a drawn random state.
   */
  virtual std::vector<float> sampleState() const = 0;

  /**
   * @return a random seed to use for intializing another Sampler.
   */
  virtual Seed sampleSeed() const = 0;
};

using SamplerPtr = std::unique_ptr<Sampler>;

/**
 * @brief A sampler drawing samples inside an n-dimensioned hypercube
 * described by 2 corners. For example, corners [l1, l2, l3, l4] and [u1, u2,
 * u3, u4], describe an hyperbox whose points [x1,x2,x3,x4] are all such that:
 * li <= xi <= ui
 */
class HyperBox : public Sampler, protected UniformEngine {
public:
  /**
   * @param the lower corner of the hyperbox
   * @param the upper corner of the hyperbox
   * @throw if lowerCorner and upperCorner size mismatch or some of the values
   * inside lowerCorner are greater than ones in upperCorner
   */
  HyperBox(const std::vector<float> &lowerCorner,
           const std::vector<float> &upperCorner,
           const std::optional<Seed> &seed = std::nullopt);

  std::unique_ptr<Sampler> copy() const override;

  std::vector<float> sampleState() const override;

  Seed sampleSeed() const final { return this->UniformEngine::sampleSeed(); }

  const auto &minCorner() const { return min_corner; }
  std::vector<float> maxCorner() const;

private:
  HyperBox(const HyperBox &o);

  std::vector<float> min_corner;
  std::vector<float> delta_corner;
};
} // namespace mt_rrt
