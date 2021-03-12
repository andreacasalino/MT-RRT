/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_STAR_SHARED_H
#define MT_RRT_TREE_STAR_SHARED_H

#include "TreeShared.h"
#include <TreeStar.h>

namespace mt::solver::shared {
	class TreeStarShared : public TreeStar<TreeShared> {
	public:
		TreeStarShared(NodePtr root, const std::vector<ProblemPtr>& problems);

		Node* add(NodePtr node) override;
	};
}

#endif