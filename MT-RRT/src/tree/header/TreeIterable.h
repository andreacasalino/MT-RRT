/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_ITERABLE_H
#define MT_RRT_TREE_ITERABLE_H

#include <TreeBase.h>

namespace mt {
	/** @brief Base Tree class physically storing the nodes
	 */
	class TreeIterable : virtual public TreeBase {
	public:
		inline Nodes::const_reverse_iterator rend() const override { return this->nodes.rend(); };
		inline Nodes::const_reverse_iterator rbegin() const override { return this->nodes.rbegin(); };
		
		/** @brief Add the passed node to the collection.
		 *  @param the node to introduce in the tree
	 	 */
		virtual Node* add(NodePtr node);

	protected:
		TreeIterable(NodePtr root);	

		/** @brief The collection of nodes
	 	 */
		Nodes nodes;
	};
}

#endif