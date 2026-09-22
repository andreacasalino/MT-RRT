/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Node.h>
#include <MT-RRT/Solution.h>
#include <MT-RRT/extend/ExtendTypes.h>

namespace mt_rrt {
template <typename T>
concept Tree = requires(const T obj_const, std::span<const float> state) {
  /**
   * @brief nullptr if nothing was found
   */
  { obj_const.nearestNeighbour(state) } -> std::same_as<const Node *>;
}
&&requires(const T obj_const, const Node &subject, NearSet &recipient) {
  /**
   * @brief nullptr if nothing was found
   */
  { obj_const.nearSet(subject, recipient) } -> std::same_as<void>;
}
&&requires(T obj, const Node &subject, NearSet &recipient) {
  /**
   * @brief nullptr if nothing was found
   */
  { obj.internalize(subject) } -> std::same_as<Node *>;
}
&&requires(T obj, const Node &new_father, const std::vector<Rewire> &rewires) {
  /**
   * @brief nullptr if nothing was found
   */
  { obj.applyRewires(new_father, rewires) } -> std::same_as<void>;
};
} // namespace mt_rrt
