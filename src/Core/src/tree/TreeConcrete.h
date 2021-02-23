/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_CONCRETE_H
#define MT_RRT_TREE_CONCRETE_H

#include <solver/Tree.h>

namespace mt::solver::tree {
	class TreeConcrete : public tree::Tree {
	public:
		const Node* extendRandom() override;

		std::pair<const Node*, bool> extendDeterministic(const NodeState& target) override;

		inline problem::Problem& getProblem() override { return this->problem; };

		inline const Node* getRoot() const override { return this->nodes.front().get(); };

		TreeConcrete(problem::Problem& problem, NodePtr root);

	protected:
		inline const Nodes& getNodes() const override { return this->nodes; };

		virtual Node* nearestNeighbour(const NodeState& state);

		problem::Problem& problem;
		Nodes nodes;
	};
}

#endif