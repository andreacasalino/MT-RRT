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
	class TreeCore 
		: public TreeIterable
		, public TreeExtendable {
	public:
		TreeCore(NodePtr root, Problem& problem);	

        Node* extendRandom();
	};
}

#endif