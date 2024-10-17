/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#include <MT-RRT/TreeHandler.h>
#include <MT-RRT/TreeUtils.h>

#include <algorithm>
#include <math.h>

namespace mt_rrt {
TreeHandlerBase::TreeHandlerBase(const ProblemDescriptionPtr &problem,
                                 const Parameters &parameters)
    : ProblemAware{problem}, parameters(parameters) {}

TreeHandler::TreeHandler(const View &root, const ProblemDescriptionPtr &problem,
                         const Parameters &parameters)
    : TreeHandlerBase(problem, parameters) {
  auto &rootNode = allocator.emplace_back(root);
  nodes.push_back(&rootNode);
}

TreeHandler::TreeHandler(Node &root, const ProblemDescriptionPtr &problem,
                         const Parameters &parameters)
    : TreeHandlerBase(problem, parameters) {
  nodes.push_back(&root);
};

const Node *TreeHandler::nearestNeighbour(const View &state) const {
  return nearest_neighbour(state, nodes.begin(), nodes.end(),
                           DescriptionAndParameters{problem(), parameters});
}

NearSet TreeHandler::nearSet(const Node &subject) const {
  NearSet res;
  res.cost2RootSubject = subject.cost2Root();
  res.set = near_set(subject.state(), nodes.begin(), nodes.end(), nodes.size(),
                     DescriptionAndParameters{problem(), parameters});
  return res;
}

Node *TreeHandler::internalize(const Node &subject) {
  auto &added = allocator.emplace_back(subject.state());
  added.setParent(*subject.getParent(), subject.cost2Go());
  nodes.push_back(&added);
  return &added;
}

void TreeHandler::applyRewires(const Node &parent, const Rewires &rewires) {
  for (auto [involved, cost] : rewires.involved_nodes) {
    involved->setParent(parent, cost);
  }
}

} // namespace mt_rrt
