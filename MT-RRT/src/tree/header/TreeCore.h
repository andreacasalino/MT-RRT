/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_CORE_H
#define MT_RRT_TREE_CORE_H

#include <Tree.h>

namespace mt {
	class TreeCore : virtual public Tree {
	public:
		TreeCore(Problem& problem, NodePtr root);	

		std::pair<NodePtr, bool> extend(const NodeState& target);

        Node* extendRandom();

		Node* add(NodePtr node) override;

		inline const Nodes& getNodes() const override { return this->nodes; };

		inline const Problem& getProblemConst() const override { return this->problem; }

	protected:
		inline Problem& getProblem() override { return this->problem; };
		
		virtual Node* nearestNeighbour(const NodeState& state) const;	

	private:
		Problem& problem;
		Nodes nodes;
	};
}

#endif