/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>
#include <MT-RRT/ExtendTypes.h>
#include <MT-RRT/Node.h>
#include <MT-RRT/NodesIterator.h>
#include <MT-RRT/Solution.h>

namespace mt_rrt {
namespace tree {
template <typename T>
concept HasIter = requires(const T obj_const) {
  typename T::iter_type;

  requires NodesIterator<typename T::iter_type>;

  { obj_const.iter() } -> std::same_as<typename T::iter_type>;
};

template <typename T, typename C>
concept HasCustomQueries = requires(const T obj_const,
                                    std::span<const float> state,
                                    const C &conn) {
  requires Connector<C>;

  { obj_const.nearestNeighbour(state, conn) } -> std::same_as<NearestNeighbour>;
}
&&requires(const T obj_const, const C &conn, NearSetHandler recipient) {
  requires Connector<C>;

  { obj_const.nearSet(recipient, conn) } -> std::same_as<void>;
};

template <typename T, typename C>
concept HasIterOrCustomQueries = HasIter<T> || HasCustomQueries<T, C>;

template <typename T, typename C>
concept HasCustomRewiring = requires(T obj, const C &conn, const Node &pivot,
                                     const std::vector<Rewire> &rew) {
  requires Connector<C>;

  { obj.applyRewiring(pivot, rew, conn) } -> std::same_as<void>;
};

template <typename T>
concept IsExtendable = requires(T obj, std::span<const float> state,
                                const Node &parent, const Positive &cost2Go) {
  { obj.internalize(state, parent, cost2Go) } -> std::same_as<const Node *>;
};
} // namespace tree
} // namespace mt_rrt
