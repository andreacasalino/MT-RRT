/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_CONCRETE_H
#define MT_RRT_TREE_CONCRETE_H

#include <Tree.h>
#include <set>

namespace mt {
	class TreeConcrete : public Tree {
	public:
		TreeConcrete(Problem& problem, NodePtr root);

		std::pair<NodePtr, bool> extend(const NodeState& target) override;

		inline void  add(NodePtr node) override { if(nullptr != node) this->nodes.emplace_back(std::move(node)); };

		inline const Nodes& getNodes() const override { return this->nodes; };

		inline Problem& getProblem() override { return this->problem; }
		inline const Problem& getProblem() const override { return this->problem; }

		std::list<Rewird> computeRewirds(Node& pivot, const Nodes::const_reverse_iterator& delimiter) const override;

		inline Nodes::const_reverse_iterator getDelimiter() const override { return this->nodes.rbegin(); };

	protected:
		virtual Node* nearestNeighbour(const NodeState& state, const Nodes::const_reverse_iterator& delimiter) const;

		virtual std::set<Node*> nearSet(const NodeState& state, const Nodes::const_reverse_iterator& delimiter) const;

		Problem& problem;
		Nodes nodes;
	};
}

#endif