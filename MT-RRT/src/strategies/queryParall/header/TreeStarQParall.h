/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_STAR_QPAR_H
#define MT_RRT_TREE_STAR_QPAR_H

#include "TreeQParall.h"
#include <TreeStar.h>

namespace mt::solver::qpar {
	class TreeStarQPar : public TreeStar<TreeQPar> {
	public:
		TreeStarQPar(NodePtr root, const std::vector<ProblemPtr>& problems);

		std::set<Node*> nearSet(const NodeState& state) const override;
	};
}

#endif