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
/*
 * @brief contains the register of nodes that were already deterministically
 * steered over a certain state
 * 
 * keys are the steered node, while the values are the states
 * toward which the node were deterministically steered
 */
class DeterministicSteerRegister {
public:
  DeterministicSteerRegister() = default;

  bool contains(const Node * node, const float* data) const {
    auto it = register_.find(node);
    return it != register_.end() && it->second.find(data) != it->second.end();
  }

  void add(const Node * node, const float* data) {
    register_[node].emplace(data);
  }

private:
  std::unordered_map<const Node *, std::unordered_set<const float *>> register_;
};

class TreeHandler : public ProblemAware {
public:
  virtual ~TreeHandler() = default;

  TreeHandler(const ProblemDescriptionPtr &problem,
              const Parameters &parameters);

  // nullptr if nothing was found
  virtual const Node *nearestNeighbour(const View &state) const = 0;

  virtual NearSet nearSet(const Node &subject) const = 0;

  virtual Node *internalize(const Node &subject) = 0;

  virtual void applyRewires(const Rewires &rewires) = 0;

  std::vector<Node *> nodes;
  Parameters parameters;
  DeterministicSteerRegister deterministic_steer_register;
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

  void applyRewires(const Rewires &rewires) override;

protected:
  NodesAllocator allocator;
};

using TreeHandlerPtr = std::unique_ptr<TreeHandler>;
} // namespace mt_rrt
