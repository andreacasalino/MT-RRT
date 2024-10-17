/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Node.h>
#include <MT-RRT/Types.h>

namespace mt_rrt {
std::vector<std::vector<float>> sequence_from_root(const Node &subject);

template <typename T> using Solutions = std::vector<T>;

template <typename T>
std::vector<std::vector<float>> materialize_best(const Solutions<T> &subject) {
  if (subject.empty()) {
    return {};
  }
  return std::max_element(subject.begin(), subject.end())->materialize();
}
} // namespace mt_rrt
