/**
 * Author:    Andrea Casalino
 * Created:   16.02.2021
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/View.h>

#include <cstring>

namespace mt_rrt {
std::vector<float> View::convert() const {
  std::vector<float> res;
  res.resize(size);
  std::memcpy(res.data(), data, size * sizeof(float));
  return res;
}
} // namespace mt_rrt
