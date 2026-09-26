/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/ExtendTypes.h>
#include <MT-RRT/Tree.h>

namespace mt_rrt {
struct Rewiring {
  Rewiring(float gamma, std::size_t state_space_size);

  void reset(std::span<const float> pivot, std::size_t tree_size);

  // TODO
  // - internally compute near set getting from outside the connector and tree,
  // then compute rewires without executing them
  // - once done, get the computed rewires and apply them (from outside)

  template <Connector C, typename T>
  void
  update(const T &tree,
         Connector &connector); // internally check it Tree has iter() or not

  // template <Connector C>
  // void updateNearSet(const Node &candidate, const C &connector) {
  //   if (connector.minCost2Go(candidate.state(), state_pivot) <= ray) {
  //     float cost2Go =
  //         connector.minCost2GoConstrained(candidate.state(), state_pivot);
  //     if (cost2Go == COST_MAX)
  //       return;
  //     set.emplace_back(NearSetElement{candidate.getParent() == nullptr,
  //                                     &candidate, candidate.cost2Root(),
  //                                     cost2Go});
  //   }
  // }

  // // // void compute_rewires(Node &candidate, NearSet &&near_set,
  // // //                      const DescriptionAndParameters &context);

  // // // // For each rewire cancidate, it applies it only if that is actually
  // beffer
  // // // // than current connections
  // // // void apply_rewires(const Node &parent, const std::vector<Rewire>
  // &rewires);

  // scratch buffers
  struct Data {
    Node pivot;
    std::vector<NearSetElement> near_set;
    std::vector<Rewire> rewires;
  };

  const auto &get() const { return data_; }

private:
  float gamma_;
  std::size_t state_space_size_;

  Data data_;
};
} // namespace mt_rrt
