/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <ExtenderUtils.h>

#include <map>
#include <unordered_set>

namespace mt_rrt {
std::vector<State> sequence_from_root(const Node &subject);

class Solution {
public:
  virtual ~Solution() = default;

  virtual std::vector<State> getSequence() const = 0;
};
using SolutionPtr = std::shared_ptr<Solution>;

using Solutions = std::multimap<float, SolutionPtr>;

void emplace_solution(PlannerSolution &recipient,
                      const SolutionPtr &solution_found);

void emplace_solution(PlannerSolution &recipient,
                      const Solutions &found_solutions);

class SingleSolution : public Solution {
public:
  SingleSolution(const Node &by_pass_node, const State &target)
      : by_pass_node(by_pass_node), target(target){};

  std::vector<State> getSequence() const final;

  const Node &by_pass_node;
  const State &target;
};

class BidirSolution : public Solution {
public:
  BidirSolution(const Node &by_pass_begin, const Node &by_pass_end)
      : by_pass_begin(by_pass_begin), by_pass_end(by_pass_end){};

  std::vector<State> getSequence() const final;

  const Node &by_pass_begin;
  const Node &by_pass_end;
};

class TreeHandler : public ProblemAware {
public:
  virtual ~TreeHandler() = default;

  TreeHandler(const NodePtr &root, const ProblemDescriptionPtr &problem,
              const Parameters &parameters);
  TreeHandler(const TreePtr &tree, const ProblemDescriptionPtr &problem,
              const Parameters &parameters);

  // nullptr if nothing was found
  virtual Node *nearestNeighbour(const State &state) {
    return nearest_neighbour(state, tree->rbegin(), tree->rend(),
                             DescriptionAndParameters{problem(), parameters});
  }

  virtual NearSet nearSet(const Node &subject) {
    return near_set(subject, tree->rbegin(), tree->rend(),
                    DescriptionAndParameters{problem(), parameters});
  }

  virtual Node *add(Node &&to_add) {
    tree->push_back(std::make_shared<Node>(std::move(to_add)));
    return tree->back().get();
  }

  virtual void applyRewires(Node &new_father, const Rewires &rewires) {
    for (const auto &rewire : rewires) {
      rewire.involved_node.setFatherInfo(
          NodeFatherInfo{&new_father, rewire.new_cost_from_father});
    }
  }

  TreePtr tree;

  const Parameters &parameters;

  // contains the register of nodes that were already deterministically
  // steered over a certain state
  //
  // keys are the steered node, while the values are the states
  // toward which the node were deterministically steered
  using DeterministicSteerRegister =
      std::unordered_map<const Node *, std::unordered_set<const State *>>;
  DeterministicSteerRegister deterministic_steer_register;
};

using TreeHandlerPtr = std::unique_ptr<TreeHandler>;

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

  /** @brief Perform the specified number of estensions on the wrapped
  tree(s).
   * This function may be called multiple times, for performing batch of
   * extensions. All the solutions found while extending are saved and stored
   in
  //  * this object.
   * @param the number of extension to perform
   */
  std::size_t search();

  virtual std::vector<Tree> dumpTrees() const = 0;

  const Solutions &getSolutions() const { return solutions; };

protected:
  Extender(const TreeHandler &handler);

  virtual void search_iteration() = 0;

  const Parameters &parameters;
  Solutions solutions;
  std::unique_ptr<DeterminismRegulator> determinism_manager;
};

using ExtenderPtr = std::unique_ptr<Extender>;

class ExtenderSingle final : public Extender {
public:
  const State &target;
  TreeHandlerPtr tree_handler;

  ExtenderSingle(TreeHandlerPtr handler, const State &target);

  std::vector<Tree> dumpTrees() const final {
    return std::vector<Tree>{*tree_handler->tree};
  };

protected:
  void search_iteration() final;

private:
  struct ExtendResult {
    std::optional<SteerResult> steer_result;
    std::optional<Rewires> rewires;
  };
  ExtendResult extend_tree(bool toward_target);
};

class ExtenderBidirectional final : public Extender {
public:
  TreeHandlerPtr front_handler;
  TreeHandlerPtr back_handler;

  ExtenderBidirectional(TreeHandlerPtr front, TreeHandlerPtr back);

  std::vector<Tree> dumpTrees() const final {
    return std::vector<Tree>{*front_handler->tree, *back_handler->tree};
  };

protected:
  void search_iteration() final;

private:
  bool extension_state = false; // if true master is front, slave is back
};
} // namespace mt_rrt
