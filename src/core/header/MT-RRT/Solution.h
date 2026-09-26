/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Node.h>
#include <MT-RRT/Types.h>

#include <optional>

namespace mt_rrt {
class Solution {
public:
  Solution(std::span<const float> start);

  void add(std::span<const float> next, Positive cost2Go);

  auto len() const { return len_; }

  auto cost() const { return cost_; }

  struct Iterator {
    Iterator(std::span<const float> rest, std::size_t state_len)
        : rest_{rest}, state_len_{state_len} {}

    std::optional<std::span<const float>> next();

  private:
    std::span<const float> rest_;
    std::size_t state_len_;
  };
  Iterator iter() const { return Iterator{states_, state_len_}; }

private:
  float cost_{0};
  std::size_t len_{0};
  std::size_t state_len_;
  std::vector<float> states_;
};
} // namespace mt_rrt
