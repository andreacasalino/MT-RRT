/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/ExtenderBidir.h>

namespace mt_rrt {
mt_rrt::Solution ExtenderBidirectionalSolution::materialize() const {
  static thread_local std::vector<const Node *> chain;
  chain.clear();
  const Node *cursor = front;
  while (cursor) {
    chain.push_back(cursor);
    cursor = cursor->data().parent;
  }

  mt_rrt::Solution res{(*chain.rbegin())->data().state};
  std::for_each(chain.rbegin() + 1, chain.rend(), [&](const Node *node) {
    res.add(node->data().state, node->data().cost2Go);
  });

  res.add(back->data().state, cost2Bridge);
  cursor = back;
  while (cursor) {
    res.add(cursor->data().state, cursor->data().cost2Go);
    cursor = cursor->data().parent;
  }
  return res;
}
} // namespace mt_rrt
