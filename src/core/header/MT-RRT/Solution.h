/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Node.h>
#include <MT-RRT/Types.h>

#include <unordered_map>

namespace mt_rrt {
std::vector<std::vector<float>> sequence_from_root(const Node &subject);

class Solution {
public:
  virtual ~Solution() = default;
  virtual std::vector<std::vector<float>> getSequence() const = 0;
  virtual float cost() const = 0;
};

using Solutions = std::vector<std::shared_ptr<Solution>>;

void sort_solutions(Solutions &subject);

std::vector<std::vector<float>> find_best_solution(const Solutions &subject);
} // namespace mt_rrt
