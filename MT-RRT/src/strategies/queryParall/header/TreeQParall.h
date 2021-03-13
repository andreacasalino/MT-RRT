/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_QPAR_H
#define MT_RRT_TREE_QPAR_H

#include <TreeCore.h>
#include "../../ProblemBattery.h"
#include "Pool.h"

namespace mt::solver::qpar {
	class TreeQPar 
		: public TreeCore
		, public ProblemBattery {
	public:
		TreeQPar(NodePtr root, const std::vector<ProblemPtr>& problems);
		TreeQPar(NodePtr root, const TreeQPar& o);

		inline void open() { this->pool->open(this->problems.size()); };
		inline void close() { this->pool->close(); };

	protected:
		Node* nearestNeighbour(const NodeState& state) const override;

		std::shared_ptr<Pool> pool;
	};
}

#endif