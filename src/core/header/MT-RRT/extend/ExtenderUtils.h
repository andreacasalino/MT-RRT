/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.hxx>
#include <MT-RRT/concepts/Tree.h>
#include <MT-RRT/extend/ExtendTypes.h>

#include <deque>

namespace mt_rrt {
template <Tree T, Connector C, bool IsDeterministic>
std::optional<SteerResult> extend(std::span<const float> target, T &tree,
                                  const C &connector);

template <Tree T, Connector C, bool IsDeterministic>
std::optional<SteerResult> extend_star(std::span<const float> target, T &tree,
                                       const C &connector,
                                       std::vector<Rewire> &rewires);

void compute_rewires(std::vector<Rewire> &recipient, Node &candidate,
                     NearSet &&near_set,
                     const DescriptionAndParameters &context);

void apply_rewires_if_better(const Node &parent,
                             const std::vector<Rewire> &rewires);
} // namespace mt_rrt
