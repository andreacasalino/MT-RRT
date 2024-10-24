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

  bool contains(const Node *node, const float *data) const {
    auto it = register_.find(node);
    return it != register_.end() && it->second.find(data) != it->second.end();
  }

  void add(const Node *node, const float *data) {
    register_[node].emplace(data);
  }

private:
  std::unordered_map<const Node *, std::unordered_set<const float *>> register_;
};

class TreeHandlerBase : public ProblemAware {
public:
  TreeHandlerBase(const ProblemDescriptionPtr &problem,
                  const Parameters &parameters);

  std::vector<Node *> nodes;
  Parameters parameters;
  DeterministicSteerRegister deterministic_steer_register;
};

class TreeHandler : public TreeHandlerBase {
public:
  TreeHandler(const View &root, const ProblemDescriptionPtr &problem,
              const Parameters &parameters);
  TreeHandler(Node &root, const ProblemDescriptionPtr &problem,
              const Parameters &parameters);

  const Node *nearestNeighbour(const View &state) const;

  NearSet nearSet(const Node &subject) const;

  Node *internalize(const Node &subject);

  void applyRewires(const Node &parent, const Rewires &rewires);

protected:
  TreeHandler(const ProblemDescriptionPtr &problem,
                  const Parameters &parameters)
  : TreeHandlerBase{problem, parameters} {}

  NodesAllocator allocator;
};

template <typename TreeT> using TreeHandlerPtr = std::unique_ptr<TreeT>;

template <typename TreeT, typename... ARGS>
TreeHandlerPtr<TreeT> make_tree(ARGS &&...args) {
  return std::make_unique<TreeT>(std::forward<ARGS>(args)...);
}
} // namespace mt_rrt
