/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/View.h>

#include <cstring>
#include <math.h>

namespace mt_rrt {
std::vector<float> View::convert() const {
  std::vector<float> res;
  res.resize(size);
  std::memcpy(res.data(), data, size * sizeof(float));
  return res;
}

float euclidean_distance(const View &a, const View &b) {
  float result = 0;
  for (std::size_t k = 0; k < a.size; ++k) {
    result += powf(a.data[k] - b.data[k], 2.f);
  }
  return sqrtf(result);
}
} // namespace mt_rrt
