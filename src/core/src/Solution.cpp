/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Solution.h>

#include <algorithm>

namespace mt_rrt {
std::optional<std::span<const float>> Solution::Iterator::next() {
  if (rest_.empty()) {
    return std::nullopt;
  }
  std::span<const float> res{rest_.begin(), rest_.begin() + state_len_};
  rest_ = {rest_.begin() + state_len_, rest_.end()};
  return res;
}

Solution::Solution(std::span<const float> start) : state_len_{start.size()} {
  add(start, 0);
  states_.insert(states_.end(), start.begin(), start.end());
  len_ += 1;
}

void Solution::add(std::span<const float> next, Positive cost2Go) {
  cost_ += cost2Go.get();
  len_ += 1;
  states_.insert(states_.end(), next.begin(), next.end());
}

Solution Solution::fromFinalState(const Node &subject) {
  Solution res{subject.data().state};
  for (const Node *current = subject.data().parent; current;
       current = current->data().parent) {
  }
  // reverse
  std::reverse(res.states_.begin(), res.states_.end());
  for (std::size_t offset{0}; offset < res.states_.size();
       offset += res.state_len_) {
    std::reverse(res.states_.begin() + offset,
                 res.states_.begin() + offset + res.state_len_);
  }
  return res;
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
      [&comparer](const auto &a, const auto &b) { return comparer(a, b); });
}

std::optional<Solution> find_best_solution(const Solutions &subject) {
  if (subject.empty()) {
    return std::nullopt;
  };

  SolutionsComparer comparer;
  auto &best = *std::min_element(
      subject.begin(), subject.end(),
      [&comparer](const auto &a, const auto &b) { return comparer(a, b); });
  return std::make_optional(best);
}
} // namespace mt_rrt
