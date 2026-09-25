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

void Solution::add(std::span<const float> next, Positive cost2Go) noexcept {
  cost_ += cost2Go.get();
  len_ += 1;
  states_.insert(states_.end(), next.begin(), next.end());
}

Solution Solution::fromTargetAndEndingState(const Node &ending,
                                            std::span<const float> target,
                                            Positive cost2Target) {
  Solution res{target};
  res.cost_ = cost2Target.get();
  for (const Node *current = &ending; current;
       current = current->data().parent) {
    res.add(current->data().state, current->cost2Root());
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

void sort_solutions(Solutions &subject) {
  std::sort(subject.begin(), subject.end(),
            [](const auto &a, const auto &b) { return a.cost() < b.cost(); });
}

std::optional<Solution> find_best_solution(const Solutions &subject) {
  auto it = std::min_element(
      subject.begin(), subject.end(),
      [](const auto &a, const auto &b) { return a.cost() < b.cost(); });

  return it == subject.end() ? std::nullopt : std::make_optional(*it);
}
} // namespace mt_rrt
