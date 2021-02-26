/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_QPAR_TREE_H
#define MT_RRT_QPAR_TREE_H

#include <TreeConcrete.h>

namespace mt::qpar {
	class Tree : public mt::TreeConcrete {
	public:
		Tree(Problem& problem, NodePtr root);

		Node* extendRandom() override;

		std::pair<Node*, bool> extendDeterministic(const NodeState& target) override;
	};

	class TreeStar : public mt::qpar::Tree {
	public:
		TreeStar(Problem& problem, NodePtr root);
	};
}

#endif