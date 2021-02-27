/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_DECORATOR_H
#define MT_RRT_TREE_DECORATOR_H

#include <Tree.h>

namespace mt {
	class TreeDecorator : public Tree {
	public:
		inline Node* extendRandom() override { return this->wrapped->extendRandom(); };

		inline std::pair<Node*, bool> extendDeterministic(const NodeState& target) override { return this->wrapped->extendDeterministic(target); };

		inline const Nodes& getNodes() const override { return this->wrapped->getNodes(); };

		inline Problem& getProblem() override { return this->wrapped->getProblem(); }

		inline Tree* get() { return this->wrapped.get(); };
		inline const Tree* get() const { return this->wrapped.get(); };

		template<typename TreeType>
		inline const TreeType* get() const { return dynamic_cast<const TreeType*>(this->wrapped.get()); }
		template<typename TreeType>
		inline TreeType* get() { return dynamic_cast<TreeType*>(this->wrapped.get()); }

	protected:
		TreeDecorator(TreePtr wrapped);

		TreePtr wrapped;
	};
}

#endif