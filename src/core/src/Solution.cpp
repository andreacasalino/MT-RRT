/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/Solution.h>

namespace mt_rrt {
std::vector<std::vector<float>> sequence_from_root(const Node &subject) {
  std::vector<const Node *> nodes;
  for (const Node *cursor = &subject; cursor; cursor = cursor->getParent()) {
    nodes.push_back(cursor);
  }
  std::vector<std::vector<float>> res;
  res.reserve(nodes.size());
  std::for_each(nodes.rbegin(), nodes.rend(), [&res](const Node *ptr) {
    res.emplace_back(ptr->state().convert());
  });
  return res;
}
} // namespace mt_rrt