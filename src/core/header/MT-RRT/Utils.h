/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>
#include <MT-RRT/Node.h>
#include <MT-RRT/Types.h>

namespace mt_rrt {
struct NearestNeighbour {
  std::span<const float> state_to_reach;
  const Node *closest = nullptr;
  float closestCost = COST_MAX;

  template <Connector C> void update(const Node &candidate, C &connector) {
    auto cost2Go =
        connector.makeTrajectory(candidate.data().state, state_to_reach)
            .minCost2Go()
            .get();
    if (cost2Go < closestCost) {
      closest = &candidate;
      closestCost = cost2Go;
    }
  }
};

struct NearSetElement {
  bool isRoot;
  const Node *element;
  Positive cost2Root;
  Positive cost2go;

  float costTot() const { return cost2Root.get() + cost2go.get(); }
};

struct NearSetHandler {
  NearSetHandler(Positive r, Node &pvt, std::vector<NearSetElement> &set);

  template <Connector C> void tryAdd(Node &node, C &connector) {
    auto traj = connector.makeTrajectory(node.data().state, pivot.data().state);
    if (traj.has_value() && traj->minCost2Go().get() <= ray) {
      auto traversed = traj->traverse();
      if (traversed.has_value()) {
        float cost2Go = traversed->cost2Go.get();
        near_set.emplace_back(NearSetElement{node.data().parent == nullptr,
                                             &node, node.cost2Root(), cost2Go});
      }
    }
  }

  Positive ray;
  Node &pivot;
  std::vector<NearSetElement> &near_set;
};

struct Rewire {
  Node *involved_node;
  Positive updatedCost2Go;
};

struct Rewiring {
  Rewiring(Positive gamma, std::size_t state_space_size);

  template <Connector C, typename T>
  void update(Node &pivot, const T &tree, C &connector);

  const auto &getRewires() const { return rewires_; }

private:
  Positive nearSetRay(std::size_t tree_size) {
    const float tree_size_float = static_cast<float>(tree_size);
    return gamma_.get() * powf(logf(tree_size_float) / tree_size_float,
                               1.f / static_cast<float>(state_space_size_));
  }

  template <Connector C>
  void computeRewires(NearSetHandler &handler, C &connector);

  Positive gamma_;
  std::size_t state_space_size_;

  // scratch buffers
  std::vector<NearSetElement> near_set_;
  std::vector<Rewire> rewires_;
};
} // namespace mt_rrt
