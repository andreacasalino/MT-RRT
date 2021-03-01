/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_QPAR_H
#define MT_RRT_TREE_QPAR_H

#include <TreeConcrete.h>
#include "Pool.h"

namespace mt::qpar {
	class TreeQPar : public TreeConcrete {
	public:
		TreeQPar(const std::vector<ProblemPtr>& problems, NodePtr root);
		TreeQPar(const TreeQPar& o, NodePtr root);

		inline void open() { this->pool->open(this->problems.size()); };
		inline void close() { this->pool->close(); };

	private:
		Node* nearestNeighbour(const NodeState& state, const Nodes::const_reverse_iterator& delimiter) const override;

		std::set<Node*> nearSet(const NodeState& state, const Nodes::const_reverse_iterator& delimiter) const override;

		std::vector<Problem*> problems;
		std::shared_ptr<Pool> pool;
	};
}

#endif