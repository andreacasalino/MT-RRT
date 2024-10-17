/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Solution.h>

#include <algorithm>

namespace mt_rrt {
std::vector<std::vector<float>> sequence_from_root(const Node &subject) {
  std::vector<std::vector<float>> result;
  for (const auto *cursor = &subject; nullptr != cursor;
       cursor = cursor->getParent()) {
    result.emplace_back(cursor->state().convert());
  }
  std::reverse(result.begin(), result.end());
  return result;
}

void sort_solutions(Solutions &subject) {
  std::unordered_map<const Solution *, float> costs;
  for(const auto& sol : subject) {
    costs.emplace(sol.get(), sol->cost());
  }
  std::sort(
      subject.begin(), subject.end(),
      [&costs](const auto &a, const auto &b) { return costs.at(a.get()) < costs.at(b.get()); });
}

std::vector<std::vector<float>> materialize_best(const Solutions &subject) {
  if (subject.empty()) {
    return {};
  };
  std::vector<float> costs;
  costs.reserve(subject.size());
  for(const auto& sol : subject) {
    costs.emplace_back(sol->cost());
  }
  std::size_t index = std::max_element(costs.begin(), costs.end()) - costs.begin();
  return subject[index]->materialize();
}
} // namespace mt_rrt
