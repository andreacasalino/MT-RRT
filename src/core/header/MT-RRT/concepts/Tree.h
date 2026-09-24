/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Node.h>
#include <MT-RRT/Solution.h>
#include <MT-RRT/concepts/NodesIterator.h>
#include <MT-RRT/extend/ExtendTypes.h>

namespace mt_rrt {
struct DeterministicSteerRegisterHash {
  template <typename T, typename U>
  std::size_t
  operator()(const std::pair<const Node *, const float *> &p) const noexcept {
    std::size_t h1 = std::hash<T *>{}(p.first);
    std::size_t h2 = std::hash<U *>{}(p.second);

    // Combine the hashes using a high-quality mixing formula (from Boost)
    // This avoids collisions like (A, B) having the same hash as (B, A) if T ==
    // U
    return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
  }
};

// contains the register of nodes that were already deterministically
// steered over a certain state
//
// keys are the steered node, while the values are the states
// toward which the node were deterministically steered
using DeterministicSteerRegister =
    std::unordered_set<std::pair<const Node *, const float *>,
                       DeterministicSteerRegisterHash>;

template <typename T>
concept Tree = requires(const T obj_const) {
  typename T::the_iter;

  requires NodesIterator<typename T::the_iter>;

  { obj_const.iter() } -> std::same_as<typename T::the_iter>;
}
&&requires(T obj, std::span<const float> state, const Node &parent,
           const Positive &cost2Go) {
  /**
   * @brief nullptr if nothing was found
   */
  { obj.internalize(state, parent, cost2Go) } -> std::same_as<const Node *>;
}
&&requires(T obj, const Rewires &rew) {
  /**
   * @brief nullptr if nothing was found
   */
  { obj.apply(rew) } -> std::same_as<void>;
}
&&requires(T obj) {
  {
    obj.getDeterministicRegister()
    } -> std::same_as<DeterministicSteerRegister &>;
};
} // namespace mt_rrt
