/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Error.h>
#include <MT-RRT/Sampler.h>

namespace mt_rrt {
HyperBox::HyperBox(const std::vector<float> &lowerCorner,
                   const std::vector<float> &upperCorner,
                   const std::optional<Seed> &seed)
    : UniformEngine(0, 1.f, seed) {
  // validate inputs
  if (lowerCorner.empty() || upperCorner.empty()) {
    throw Error{"corners can't be empty"};
  }
  if (lowerCorner.size() != upperCorner.size()) {
    throw Error{"corners should have the same size"};
  }
  min_corner = lowerCorner;
  delta_corner = upperCorner;
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

std::unique_ptr<Sampler> HyperBox::copy() const {
  std::unique_ptr<HyperBox> result;
  result.reset(new HyperBox{*this});
  return result;
}

std::vector<float> HyperBox::sampleState() const {
  std::vector<float> result;
  result.reserve(min_corner.size());
  for (std::size_t k = 0; k < min_corner.size(); ++k) {
    result.push_back(min_corner[k] + UniformEngine::sample() * delta_corner[k]);
  }
  return result;
}

std::vector<float> HyperBox::maxCorner() const {
  std::vector<float> result = min_corner;
  for (std::size_t k = 0; k < result.size(); ++k) {
    result[k] += delta_corner[k];
  }
  return result;
}
} // namespace mt_rrt