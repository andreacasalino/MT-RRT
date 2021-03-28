/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_SLAVE_H
#define MT_RRT_TREE_SLAVE_H

#include <TreeCore.h>

namespace mt::solver::multiag {
	class TreeSlave : public TreeCore {
	public:
		TreeSlave(Problem& problem);

		inline Nodes* getNodes() { return &this->nodes; }
	};
}

#endif