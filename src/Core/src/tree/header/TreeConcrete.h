/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_CONCRETE_H
#define MT_RRT_TREE_CONCRETE_H

#include <solver/Tree.h>
#include <problem/Problem.h>
#include <set>

namespace mt::solver::tree {
	class TreeConcrete : public tree::Tree {
	public:
		TreeConcrete(problem::Problem& problem, NodePtr root);

		const Node* extendRandom() override;

		std::pair<const Node*, bool> extendDeterministic(const NodeState& target) override;

		inline const Nodes& getNodes() const override { return this->nodes; };

		struct Rewird {
			Rewird(Node& involved, Node& newFather, const float& newCostFromFather);

			Node& involved;
			Node& newFather;
			float newCostFromFather;
		};
		std::list<Rewird> computeRewirds(Node& pivot) const;

	protected:
		virtual Node* nearestNeighbour(const NodeState& state) const;

		virtual std::set<Node*> nearSet(Node& node) const;

		problem::Problem& problem;

	private:
		Nodes nodes;
	};
}

#endif