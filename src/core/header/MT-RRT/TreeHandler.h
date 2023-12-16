/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#pragma once

#include <MT-RRT/ProblemDescription.h>

#include <unordered_map>
#include <unordered_set>

namespace mt_rrt {
struct NearSetElement {
  bool isRoot;
  Node *element;
  float cost2Root;
  float cost2go;
};

struct NearSet {
  float cost2RootSubject;
  std::vector<NearSetElement> set;
};

struct Rewire {
  Node *involved_node;
  float new_cost_from_father;
};

class TreeHandler : public ProblemAware {
public:
  virtual ~TreeHandler() = default;

  // nullptr if nothing was found
  virtual const Node *nearestNeighbour(const View &state) const = 0;

  virtual NearSet nearSet(const Node &subject) const = 0;

  virtual Node *internalize(const Node &subject) = 0;

  virtual void applyRewires(const Node &new_father,
                            const std::vector<Rewire> &rewires) = 0;

  std::vector<Node *> nodes;
  Parameters parameters;
  // contains the register of nodes that were already deterministically
  // steered over a certain state
  //
  // keys are the steered node, while the values are the states
  // toward which the node were deterministically steered
  using DeterministicSteerRegister =
      std::unordered_map<const Node *, std::unordered_set<const float *>>;
  DeterministicSteerRegister deterministic_steer_register;

protected:
  TreeHandler(const ProblemDescriptionPtr &problem,
              const Parameters &parameters);
};

class TreeHandlerBasic : public TreeHandler {
public:
  TreeHandlerBasic(const View &root, const ProblemDescriptionPtr &problem,
                   const Parameters &parameters);
  TreeHandlerBasic(Node &root, const ProblemDescriptionPtr &problem,
                   const Parameters &parameters);

  const Node *nearestNeighbour(const View &state) const override;

  NearSet nearSet(const Node &subject) const override;

  Node *internalize(const Node &subject) override;

  void applyRewires(const Node &new_father,
                    const std::vector<Rewire> &rewires) override;

protected:
  NodesAllocator allocator;
};

using TreeHandlerPtr = std::unique_ptr<TreeHandler>;

float near_set_ray(std::size_t tree_size, std::size_t problem_size,
                   float gamma);
} // namespace mt_rrt
