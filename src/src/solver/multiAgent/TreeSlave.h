/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_SLAVE_H
#define MT_RRT_TREE_SLAVE_H

#include <TreeConcrete.h>

namespace mt::multiag {
	class TreeSlave : public TreeConcrete {
	public:
		TreeSlave(Problem& problem);

		class TreeMaster;

		void  add(NodePtr node) override;

	private:
		inline Nodes& getSlaveNodes() { return this->nodes; };

		Node* originalRoot = nullptr;
	};
}

#endif