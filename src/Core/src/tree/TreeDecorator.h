/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_DECORATOR_H
#define MT_RRT_TREE_DECORATOR_H

#include <solver/Tree.h>

namespace mt::solver::tree {
	class TreeDecorator : public tree::Tree {
	public:
		inline const Node* extendRandom() override { return this->wrapped->extendRandom(); };

		inline std::pair<const Node*, bool> extendDeterministic(const NodeState& target) override { return this->wrapped->extendDeterministic(target); };

		inline problem::Problem& getProblem() override { return this->wrapped->getProblem(); };

		inline const Node* getRoot() const override { return this->wrapped->getRoot(); };

	protected:
		TreeDecorator(TreePtr wrapped);

		inline const Nodes& getNodes() const override { return getNodesOther(*this->wrapped.get()); };

		TreePtr wrapped;
	};
}

#endif