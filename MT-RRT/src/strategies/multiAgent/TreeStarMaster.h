/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_STAR_MASTER_H
#define MT_RRT_TREE_STAR_MASTER_H

#include "TreeMaster.h"

namespace mt::multiag {
	class TreeStarMaster : public TreeSlave::TreeMaster {
	public:
		TreeStarMaster(const std::vector<ProblemPtr>& problems, NodePtr root);

        void gather() override;
	};
}

#endif