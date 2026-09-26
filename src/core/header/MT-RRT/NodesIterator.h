/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Node.h>

namespace mt_rrt {
template <typename N>
concept NodesIterator = requires(N obj) {
  /**
   * @brief nullptr if end is reached
   */
  { obj.next() } -> std::same_as<const Node &>;
}
&&requires(const N obj) {
  /**
   * @brief nullptr if end is reached
   */
  { obj.size() } -> std::same_as<std::size_t>;
};

template <NodesIterator It, typename Pred>
void for_each_nodes(It it, Pred pred) {
  while (true) {
    if (const Node &next = it.next(); next) {
      pred(next);
    } else {
      break;
    }
  }
}
} // namespace mt_rrt
