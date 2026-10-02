/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Connector.h>
#include <MT-RRT/Tree.h>
#include <MT-RRT/Types.h>

namespace mt_rrt {
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

  template <Connector C> void computeRewires(Node &pivot, C &connector);

  Positive gamma_;
  std::size_t state_space_size_;

  // scratch buffers
  std::vector<NearSetElement> near_set_;
  std::vector<Rewire> rewires_;
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <Connector C, typename T>
void Rewiring::update<C, T>(Node &pivot, const T &tree, C &connector) {
  rewires_.clear();
  NearSetHandler handler{nearSetRay(tree.size()), pivot, near_set_};

  if constexpr (HasCustomQueries<T, C>) {
    tree.nearSet(handler, connector);
  } else {
    for_each_nodes(tree.iter(),
                   [&](const Node &next) { handler.tryAdd(next, connector); });
  }

  computeRewires(pivot, connector);
}

template <Connector C>
void Rewiring::computeRewires<C>(Node &pivot, C &connector) {
  // std::vector<Rewire> compute_rewires(Node &subject, NearSet
  // &&near_set_info,
  //                                     const DescriptionAndParameters
  //                                     &context)
  //                                     {
  //   auto &near_set = near_set_info.set;
  //   if (near_set.empty()) {
  //     return {};
  //   }

  //   const auto &connector = *context.description.connector;
  //   float cost2RootSubject = near_set_info.cost2RootSubject;

  //   // rewire just_steered to the best father
  //   if (auto it = std::min_element(
  //           near_set.begin(), near_set.end(),
  //           [](const NearSetElement &a, const NearSetElement &b) {
  //             return a.cost2go + a.cost2Root < b.cost2go + b.cost2Root;
  //           });
  //       it->cost2go + it->cost2Root < cost2RootSubject) {
  //     subject.setParent(*it->element, it->cost2go);
  //     cost2RootSubject = it->cost2go + it->cost2Root;
  //   }
  //   // remove current parent from rewire candidates
  //   if (auto it =
  //           std::find_if(near_set.begin(), near_set.end(),
  //                        [parent = subject.getParent()](const
  //                        NearSetElement &e) {
  //                          return e.element == parent;
  //                        });
  //       it != near_set.end()) {
  //     near_set.erase(it);
  //   }

  //   // check for rewires
  //   bool symmetric = context.description.simmetry;
  //   std::vector<Rewire> res;
  //   for (auto [isRoot, node, nodeCost2Root, cost2GoPrev] : near_set) {
  //     if (isRoot) {
  //       // root can't be rewired
  //       continue;
  //     }
  //     float cost2Go = symmetric ? cost2GoPrev
  //                               :
  //                               connector.minCost2GoConstrained(subject.state(),
  //                                                                 node->state());
  //     if (cost2Go == COST_MAX) {
  //       continue;
  //     }
  //     float cost2RootRewire = cost2RootSubject + cost2Go;
  //     if (cost2RootRewire < nodeCost2Root) {
  //       res.emplace_back(Rewire{node, cost2Go});
  //     }
  //   }
  //   return res;
  // }

  // void apply_rewires_if_better(const Node &parent,
  //                              const std::vector<Rewire> &rewires) {
  //   float parentCost2Root = parent.cost2Root();
  //   for (const auto &rew : rewires) {
  //     if (parentCost2Root + rew.new_cost_from_father <
  //         rew.involved_node->cost2Root()) {
  //       rew.involved_node->setParent(parent, rew.new_cost_from_father);
  //     }
  //   }
  // }
}

} // namespace mt_rrt
