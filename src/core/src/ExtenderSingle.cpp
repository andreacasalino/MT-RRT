/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/ExtenderSingle.h>

namespace mt_rrt {
mt_rrt::Solution ExtenderSingleSolution::materialize() const {
  static thread_local std::vector<const Node *> chain;
  chain.clear();
  const Node *cursor = pivot;
  while (cursor) {
    chain.push_back(cursor);
    cursor = cursor->data().parent;
  }

  mt_rrt::Solution res{(*chain.rbegin())->data().state};
  std::for_each(chain.rbegin() + 1, chain.rend(), [&](const Node *node) {
    res.add(node->data().state, node->data().cost2Go);
  });
  res.add(target, cost2Target);
  return res;
}
} // namespace mt_rrt
