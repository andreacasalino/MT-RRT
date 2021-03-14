/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_STAR_MASTER_H
#define MT_RRT_TREE_STAR_MASTER_H

#include "TreeMaster.h"
#include <TreeRewirer.h>

namespace mt::solver::multiag {
	class TreeStarMaster 
		: public TreeMaster 
		, public TreeRewirer {
	public:
		TreeStarMaster(NodePtr root, const std::vector<ProblemPtr>& problems);

        void gather() override;

	protected:
		std::set<Node*> nearSet(const NodeState& state) const override;

		std::vector<std::list<NodePtr>> temporaryBuffer;
	};
}

#endif