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
  if (next.size() != state_len_) {
    throw Error{"Invalid solution step space size"};
  }
  cost_ += cost2Go.get();
  len_ += 1;
  states_.insert(states_.end(), next.begin(), next.end());
}
} // namespace mt_rrt
