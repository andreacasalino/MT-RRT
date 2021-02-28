/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_STAR_H
#define MT_RRT_TREE_STAR_H

#include <TreeConcrete.h>
#include <TreeDecorator.h>

namespace mt {
	class TreeStar : public TreeDecorator {
	public:
		TreeStar(std::unique_ptr<TreeConcrete> wrapped);

		std::pair<Node*, NodePtr> extend(const NodeState& target) override;
	};
}

#endif