/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_CORE_H
#define MT_RRT_TREE_CORE_H

#include <TreeIterable.h>
#include <TreeExtendable.h>

namespace mt {
	/** @brief Tree with the minimal functionalities required to implement an rrt algorithm
	 */
	class TreeCore 
		: public TreeIterable
		, public TreeExtendable {
	public:
		TreeCore(NodePtr root, Problem& problem);	

		/** @brief sample a random state using the sampler of the problem and call extend(...)
		 * toward the sampled state. In case the extension was possible, the extended node is
		 * directly added to the tree and a pointer to it is returned.
		 *  @return the added node, in case the extension was possible.
		 */
        Node* extendRandom();
	};
}

#endif