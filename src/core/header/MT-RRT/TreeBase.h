/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/Solution.h>
#include <MT-RRT/concepts/Connector.h>
#include <MT-RRT/concepts/Sampler.h>

#include <optional>

namespace mt_rrt {
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
  Nodes nodes_;
  DeterministicSteerRegister deterministic_steers_;
};

/** @brief Used to extend one or two connected search trees
 */
// template <typename E>
// concept Extender = requires(const E obj_const) {
//   // TODO
// };

// class Extender : public ProblemAware {
// public:
//   virtual ~Extender() = default;

//   /** @brief Perform the specified number of estensions on the wrapped
//   tree(s).
//    * This function may be called multiple times, for performing batch of
//    * extensions. All the solutions found while extending are saved and stored
//    in
//    * this object.
//    * @param the number of extension to perform
//    */
//   std::size_t search();

//   virtual std::vector<TreeHandlerPtr> dumpTrees() = 0;

//   const Solutions &getSolutions() const { return solutions; };
//   Solutions &getSolutions() { return solutions; };

// protected:
//   Extender(const TreeHandler &handler);

//   virtual void search_iteration() = 0;

//   const Parameters &parameters;
//   Solutions solutions;
//   std::optional<DeterminismRegulator> determinism_manager;
// };
} // namespace mt_rrt
