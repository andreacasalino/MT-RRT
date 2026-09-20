/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Error.h>
#include <MT-RRT/Sampler.h>

namespace mt_rrt {
HyperBox::HyperBox(std::vector<float> lowerCorner,
                   std::vector<float> upperCorner,
                   std::optional<Seed> seed = std::nullopt)
    : UniformEngine(0, 1.f, seed), min_corner{std::move(lowerCorner)},
      delta_corner{std::move(upperCorner)} {
  // validate inputs
  if (lowerCorner.empty() || upperCorner.empty()) {
    throw Error{"corners can't be empty"};
  }
  if (lowerCorner.size() != upperCorner.size()) {
    throw Error{"corners should have the same size"};
  }
  for (std::size_t k = 0; k < lowerCorner.size(); ++k) {
    delta_corner[k] -= min_corner[k];
    if (delta_corner[k] < 0) {
      throw Error{"invalid corners"};
    }
  }
}

HyperBox::HyperBox(const HyperBox &o)
    : UniformEngine(o), min_corner(o.min_corner), delta_corner(o.delta_corner) {
}

void HyperBox::sampleState(std::vector<float> &recipient) const noexcept {
  recipient.clear();
  for (std::size_t k = 0; k < min_corner.size(); ++k) {
    recipient.push_back(min_corner[k] +
                        UniformEngine::sample() * delta_corner[k]);
  }
}

std::vector<float> HyperBox::maxCorner() const noexcept {
  std::vector<float> result = min_corner;
  for (std::size_t k = 0; k < result.size(); ++k) {
    result[k] += delta_corner[k];
  }
  return result;
}
} // namespace mt_rrt