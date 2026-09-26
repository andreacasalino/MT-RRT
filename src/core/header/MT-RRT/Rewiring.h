/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>
#include <MT-RRT/ExtendTypes.h>
#include <MT-RRT/Tree.h>

namespace mt_rrt {
struct Rewiring {
  Rewiring(float gamma, std::size_t state_space_size);

  template <Connector C, typename T>
  void update(Node &pivot, const T &tree, C &connector) {
    pivot_ = pivot;

    NearSetHandler ns_hndlr{...};
    if constexpr (HasCustomQueries<T, C>) {
      tree.nearSet(ns_hndlr, connector);
    } else {
      // TODO iter the tree and compute the near set
    }

    rewires_.clear();
    computeRewires(pivot, connector);

    if constexpr (HasCustomRewiring<T, C>) {
      tree.applyRewiring(pivot, rewires_, connector);
    } else {
      // TODO apply rewires one by one
    }
  }

  const auto &getRewires() const { return rewires_; }

private:
  template <Connector C> void computeRewires(Node &pivot, C &connector);

  float gamma_;
  std::size_t state_space_size_;

  // scratch buffers
  std::vector<NearSetElement> near_set_;
  std::vector<Rewire> rewires_;
};
} // namespace mt_rrt
