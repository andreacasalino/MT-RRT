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
  result.emplace_back(subject.state().convert());
  for (const auto *cursor = subject.getParent(); nullptr != cursor;
       cursor = cursor->getParent()) {
    result.emplace_back(cursor->state().convert());
  }
  std::reverse(result.begin(), result.end());
  return result;
}

namespace {
struct SolutionsComparer {
  SolutionsComparer() = default;

  bool operator()(const Solution &a, const Solution &b) const {
    return getCost(a) < getCost(b);
  }

  float getCost(const Solution &subject) const {
    auto it = solutions.find(&subject);
    if (it == solutions.end()) {
      it = solutions.emplace(&subject, subject.cost()).first;
    }
    return it->second;
  }

private:
  mutable std::unordered_map<const Solution *, float> solutions;
};
} // namespace

void sort_solutions(Solutions &subject) {
  SolutionsComparer comparer;
  std::sort(
      subject.begin(), subject.end(),
      [&comparer](const auto &a, const auto &b) { return comparer(*a, *b); });
}

std::vector<std::vector<float>> find_best_solution(const Solutions &subject) {
  if (subject.empty()) {
    return {};
  };

  SolutionsComparer comparer;
  auto &best = *std::min_element(
      subject.begin(), subject.end(),
      [&comparer](const auto &a, const auto &b) { return comparer(*a, *b); });
  return best->getSequence();
}
} // namespace mt_rrt
