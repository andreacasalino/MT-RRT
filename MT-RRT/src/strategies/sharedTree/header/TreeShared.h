/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_SHARED_H
#define MT_RRT_TREE_SHARED_H

#include <TreeCore.h>
#include "../../ProblemBattery.h"
#include <mutex>

namespace mt::solver::shared {
	class TreeShared 
		: public TreeCore 
		, public ProblemBattery {
	public:
		TreeShared(NodePtr root, const std::vector<ProblemPtr>& problems);

		Node* add(NodePtr node) override;

		Nodes::const_reverse_iterator rbegin() const override;

		Problem* getProblem() const override;
		
	protected:
		mutable std::mutex mtx;
	};
}

#endif