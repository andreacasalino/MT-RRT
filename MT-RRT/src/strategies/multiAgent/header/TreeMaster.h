/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_MASTER_H
#define MT_RRT_TREE_MASTER_H

#include <TreeCore.h>
#include "TreeSlave.h"

namespace mt::solver::multiag {
	class TreeMaster : public TreeCore {
	public:
		TreeMaster(NodePtr root, const std::vector<ProblemPtr>& problems);

		void dispatch();

		virtual void gather();

		inline Tree& getSlave(const std::size_t& pos) { return *this->slaves[pos].get(); };

	protected:
		std::vector<std::unique_ptr<TreeSlave>> slaves;
	};
}

#endif