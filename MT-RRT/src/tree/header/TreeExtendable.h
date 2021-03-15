/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_EXTENDABLE_H
#define MT_RRT_TREE_EXTENDABLE_H

#include <TreeBase.h>

namespace mt {
	class TreeExtendable : virtual public TreeBase {
	public:
		std::pair<NodePtr, bool> extend(const NodeState& target);
		
	protected:
		TreeExtendable() = default;	

		virtual Node* nearestNeighbour(const NodeState& state) const;	
	};
}

#endif