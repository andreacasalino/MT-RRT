/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT-carpet/Copiable.h>
#include <MT-RRT-carpet/Random.h>
#include <MT-RRT-core/Node.h>

namespace mt_rrt {
/** @brief Interface for a sampler of states.
 */
class Sampler : public Copiable<Sampler> {
public:
  virtual ~Sampler() = default;

  /** @brief Returns a node having a state randomly sampled in the \mathcal{X}
   * space, Sections 1.2.1, 1.2.2 and 1.2.3 of the documentation. This function
   * is invoked mainly for randomly growing a searching tree.
   * @return a drawn random state.
   */
  virtual State sampleState() const = 0;

  virtual Seed sampleSeed() const = 0;
};

using SamplerPtr = std::unique_ptr<Sampler>;

/** @brief A sampler drawing a sample inside an hypercube of n-dimensions,
 * described by 2 corners. For example, corners [l1, l2, l3, l4] and [u1, u2,
 * u3, u4], describe an hyperbox whose points [x1,x2,x3,x4] are all such that:
 * li <= xi <= ui
 */
class HyperBox : public Sampler, protected UniformEngine {
public:
  /** @param the lower corner of the hyperbox
   *  @param the upper corner of the hyperbox
   *  @throw if lowerCorner and upperCorner size mismatch or some of the values
   * inside lowerCorner are greater than ones in upperCorner
   */
  HyperBox(const State &lowerCorner, const State &upperCorner,
           const std::optional<Seed> &seed = std::nullopt);

  std::unique_ptr<Sampler> copy() const override;

  State sampleState() const override;

  Seed sampleSeed() const final { return this->UniformEngine::sampleSeed(); }

  const State &minCorner() const { return min_corner; }
  State maxCorner() const;

private:
  HyperBox(const HyperBox &o);

  State min_corner;
  State delta_corner;
};
} // namespace mt_rrt
