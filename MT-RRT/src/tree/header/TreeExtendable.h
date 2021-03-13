/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_EXTENDABLE_H
#define MT_RRT_TREE_EXTENDABLE_H

#include <Tree.h>

namespace mt {
	class TreeExtendable : virtual public Tree {
	public:
		std::pair<NodePtr, bool> extend(const NodeState& target);

        Node* extendRandom();

	protected:
		TreeExtendable() = default;	

		virtual Node* nearestNeighbour(const NodeState& state) const;	
	};
}

#endif