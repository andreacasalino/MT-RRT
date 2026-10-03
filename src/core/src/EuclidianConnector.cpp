/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/EuclidianConnector.h>

namespace mt_rrt {
float euclidean_distance(std::span<const float> a, std::span<const float> b) {
  float res{0};
  for (int k = 0; k < a.size(); ++k) {
    float diff = a[k] - b[k];
    res += diff * diff;
  }
  return std::sqrt(res);
}
} // namespace mt_rrt
