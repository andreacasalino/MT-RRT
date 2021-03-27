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
	/** @brief Base class for an extendable tree, i.e. a tree whose nodes can be incremented over time
	 */
	class TreeExtendable : virtual public TreeBase {
	public:
		/** @brief tried to extend the tree toward the target, performing a steering operation, Section 1.2 of the documentation
		 *  @return <a, b>:
		 * 					a: the node containing an extended node, having a father contained in this tree.
		 * 					   In case the steering procedure was not possible, a nullptr is returned.
		 * 					b: a boolean that is true in case the extension was possible AND the target was reached.
		 * 					   Otherwise is false.
	 	 */
		std::pair<NodePtr, bool> extend(const NodeState& target);
		
	protected:
		TreeExtendable() = default;	

		virtual Node* nearestNeighbour(const NodeState& state) const;	
	};
}

#endif