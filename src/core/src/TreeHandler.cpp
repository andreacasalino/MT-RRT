/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/TreeHandler.h>

#include <MT-RRT/ExtenderUtils.h>

#include <algorithm>
#include <math.h>

namespace mt_rrt {
TreeHandler::TreeHandler(const ProblemDescriptionPtr &problem,
                         const Parameters &parameters)
    : ProblemAware{problem}, parameters(parameters) {}

TreeHandlerBasic::TreeHandlerBasic(const View &root,
                                   const ProblemDescriptionPtr &problem,
                                   const Parameters &parameters)
    : TreeHandler(problem, parameters) {
  auto &rootNode = allocator.emplace_back(root);
  nodes.push_back(&rootNode);
}

TreeHandlerBasic::TreeHandlerBasic(Node &root,
                                   const ProblemDescriptionPtr &problem,
                                   const Parameters &parameters)
    : TreeHandler(problem, parameters) {
  nodes.push_back(&root);
};

const Node *TreeHandlerBasic::nearestNeighbour(const View &state) const {
  return nearest_neighbour(state, nodes.begin(), nodes.end(),
                           DescriptionAndParameters{problem(), parameters});
}

NearSet TreeHandlerBasic::nearSet(const Node &subject) const {
  NearSet res;
  res.cost2RootSubject = subject.cost2Root();
  res.set = near_set(subject.state(), nodes.begin(), nodes.end(), nodes.size(),
                     DescriptionAndParameters{problem(), parameters});
  return res;
}

Node *TreeHandlerBasic::internalize(const Node &subject) {
  auto &added = allocator.emplace_back(subject.state());
  added.setParent(*subject.getParent(), subject.cost2Go());
  nodes.push_back(&added);
  return &added;
}

void TreeHandlerBasic::applyRewires(const Node &new_father,
                                    const std::vector<Rewire> &rewires) {
  for (const auto &rewire : rewires) {
    rewire.involved_node->setParent(new_father, rewire.new_cost_from_father);
  }
}

} // namespace mt_rrt
