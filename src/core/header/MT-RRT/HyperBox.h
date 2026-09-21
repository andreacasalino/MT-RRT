/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Random.h>

#include <memory>
#include <optional>
#include <vector>

namespace mt_rrt {
/**
 * @brief A sampler drawing samples inside an n-dimensioned hypercube
 * described by 2 corners. For example, corners [l1, l2, l3, l4] and [u1, u2,
 * u3, u4], describe an hyperbox whose points [x1,x2,x3,x4] are all such that:
 * li <= xi <= ui
 */
class HyperBox : protected UniformEngine {
public:
  /**
   * @param the lower corner of the hyperbox
   * @param the upper corner of the hyperbox
   * @throw if lowerCorner and upperCorner size mismatch or some of the values
   * inside lowerCorner are greater than ones in upperCorner
   */
  HyperBox(std::vector<float> lowerCorner, std::vector<float> upperCorner,
           std::optional<Seed> seed = std::nullopt);

  std::unique_ptr<HyperBox> copy() const {
    return std::make_unique<HyperBox>(*this);
  }

  void sampleState(std::vector<float> &recipient) const noexcept;

  Seed sampleSeed() const noexcept { return this->UniformEngine::sampleSeed(); }

  const auto &minCorner() const { return min_corner; }
  std::vector<float> maxCorner() const noexcept;

private:
  std::vector<float> min_corner;
  std::vector<float> delta_corner;
};
} // namespace mt_rrt
