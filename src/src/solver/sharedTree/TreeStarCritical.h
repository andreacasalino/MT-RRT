/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_STAR_CRITICAL_H
#define MT_RRT_TREE_STAR_CRITICAL_H

#include "TreeConcreteCritical.h"

namespace mt::shared {
	class TreeStarCritical : public TreeConcreteCritical {
	public:
		TreeStarCritical(const std::vector<ProblemPtr>& problems, NodePtr root);

		std::pair<NodePtr, bool> extend(const NodeState& target) override;
	};
}

#endif