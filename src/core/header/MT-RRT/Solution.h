/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Node.h>
#include <MT-RRT/Types.h>

#include <algorithm>

namespace mt_rrt {
std::vector<std::vector<float>> sequence_from_root(const Node &subject);

template <typename T> using Solutions = std::vector<T>;

template <typename T>
std::vector<std::vector<float>> materialize_best(const Solutions<T> &subject) {
  if (subject.empty()) {
    return {};
  }
  const T *best_solution = &subject.front();
  float best_cost = best_solution->cost();
  std::for_each(subject.begin() + 1, subject.end(), [&](const T &sol) {
    float cost = sol.cost();
    if (cost < best_cost) {
      best_cost = cost;
      best_solution = &sol;
    }
  });
  return best_solution->materialize();
}
} // namespace mt_rrt
