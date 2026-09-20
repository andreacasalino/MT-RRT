/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Node.h>
#include <MT-RRT/ProblemDescription.h>
#include <MT-RRT/Solution.h>

#include <algorithm>
#include <atomic>
#include <optional>
#include <unordered_set>

namespace mt_rrt {
struct NearSetElement {
  bool isRoot;
  Node *element;
  Positive cost2Root;
  Positive cost2go;
};

struct NearSet {
  Positive cost2RootSubject;
  std::vector<NearSetElement> set;
};

struct Rewire {
  Node *involved_node;
  Positive new_cost_from_father;
};

template <typename T>
concept TreeHandler = requires(const T obj_const,
                               std::span<const float> state) {
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

struct TreeData {
  Nodes<Node> nodes;

  // contains the register of nodes that were already deterministically
  // steered over a certain state
  //
  // keys are the steered node, while the values are the states
  // toward which the node were deterministically steered
  using DeterministicSteerRegister =
      std::unordered_set<std::pair<const Node *, const float *>>;
  DeterministicSteerRegister deterministic_steer_register;
};

template <Connector C, Sampler S>
class TreeHandlerBasic : public ProblemAware<C, S> {
public:
  TreeHandlerBasic(std::span<const float> root,
                   ProblemDescriptionPtr<C, S> problem)
      : ProblemAware<C, S>{problem} {
    data_.nodes.push(root);
  }

  const Node *nearestNeighbour(std::span<const float> state) const;

  void nearSet(const Node &subject, NearSet &recipient) const;

  Node *internalize(const Node &subject);

  void applyRewires(const Node &new_father, const std::vector<Rewire> &rewires);

protected:
  TreeData data_;
};

float near_set_ray(std::size_t tree_size, std::size_t problem_size,
                   float gamma);

struct NearestQuery {
  const Node *closest = nullptr;
  float closestCost = COST_MAX;

  void operator()(const Node &candidate, float cost2Go) {
    if (cost2Go < closestCost) {
      closest = &candidate;
      closestCost = cost2Go;
    }
  }
};

template <Connector C>
const Node *
nearest_neighbour(std::span<const float> state,
                  typename std::deque<Node>::const_iterator nodes_begin,
                  typename std::deque<Node>::const_iterator nodes_end,
                  const C &connector) {
  NearestQuery query;
  std::for_each(nodes_begin, nodes_end, [&](const auto &node) {
    query(node, connector.minCost2Go(node.data().state, state).get());
  });
  return query.closest;
}

template <Connector C> struct NearSetQuery {
  float ray;
  std::span<const float> state_pivot;
  const C &connector;
  NearSet &recipient;

  void operator()(Node &subject) {
    if (connector->minCost2Go(subject.state(), state_pivot) <= ray) {
      float cost2Go =
          connector->minCost2GoConstrained(subject.state(), state_pivot);
      if (cost2Go == COST_MAX)
        return;
      set.emplace_back(NearSetElement{subject.getParent() == nullptr, &subject,
                                      subject.cost2Root(), cost2Go});
    }
  }
};

template <Connector C>
void near_set(std::span<const float> state,
              typename std::deque<Node>::iterator nodes_begin,
              typename std::deque<Node>::iterator nodes_end, std::size_t size,
              C &connector, NearSet &recipient, const Positive &gamma) {
  recipient.set.clear();
  float ray = near_set_ray(size, state.size(), gamma.get());
  NearSetQuery res{ray, state, connector, recipient};
  std::for_each(nodes_begin, nodes_end, [](auto &node) {
    res(node);
  });
}

std::vector<Rewire> compute_rewires(Node &candidate, NearSet &&near_set,
                                    const DescriptionAndParameters &context);

std::optional<Connector::SteerResult> extend(const View &target,
                                             TreeHandler &tree_handler,
                                             const bool is_deterministic);

std::optional<Connector::SteerResult> extend_star(const View &target,
                                                  TreeHandler &tree_handler,
                                                  const bool is_deterministic,
                                                  std::vector<Rewire> &rewires);

void apply_rewires_if_better(const Node &parent,
                             const std::vector<Rewire> &rewires);

struct KeepSearchPredicate {
  bool best_effort;
  std::size_t max_iterations;
  ExpansionStrategy strategy;
  std::atomic_bool one_solution_was_found = false;

  bool operator()(std::size_t iter) const;
};

class DeterminismRegulator {
public:
  DeterminismRegulator(const Seed &seed, const Determinism &determinism);

  bool doDeterministicExtension() const {
    return deterministic_rate_sampler.sample() <=
           deterministic_rate_sampler_threshold;
  }

private:
  UniformEngine deterministic_rate_sampler;
  const float deterministic_rate_sampler_threshold;
};

/** @brief Used to extend one or two connected search trees
 */
class Extender : public ProblemAware {
public:
  virtual ~Extender() = default;

  /** @brief Perform the specified number of estensions on the wrapped tree(s).
   * This function may be called multiple times, for performing batch of
   * extensions. All the solutions found while extending are saved and stored in
   * this object.
   * @param the number of extension to perform
   */
  std::size_t search();

  virtual std::vector<TreeHandlerPtr> dumpTrees() = 0;

  const Solutions &getSolutions() const { return solutions; };
  Solutions &getSolutions() { return solutions; };

protected:
  Extender(const TreeHandler &handler);

  virtual void search_iteration() = 0;

  const Parameters &parameters;
  Solutions solutions;
  std::optional<DeterminismRegulator> determinism_manager;
};

using ExtenderPtr = std::unique_ptr<Extender>;
} // namespace mt_rrt
