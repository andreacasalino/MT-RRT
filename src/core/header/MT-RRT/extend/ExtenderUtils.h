/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.hxx>
#include <MT-RRT/extend/ExtendTypes.h>

#include <deque>

namespace mt_rrt {
std::optional<SteerResult> extend(const View &target, TreeHandler &tree_handler,
                                  const bool is_deterministic);

std::optional<SteerResult> extend_star(const View &target,
                                       TreeHandler &tree_handler,
                                       const bool is_deterministic,
                                       std::vector<Rewire> &rewires);

std::vector<Rewire> compute_rewires(Node &candidate, NearSet &&near_set,
                                    const DescriptionAndParameters &context);

void apply_rewires_if_better(const Node &parent,
                             const std::vector<Rewire> &rewires);
} // namespace mt_rrt
