/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_SHARED_H
#define MT_RRT_TREE_SHARED_H

#include <TreeCore.h>
#include "../../Commons.h"
#include <mutex>

namespace mt::solver::shared {
	class TreeShared 
		: public TreeCore {
	public:
		TreeShared(NodePtr root, const std::vector<ProblemPtr>& problems);

		Node* add(NodePtr node) override;

		Nodes::const_reverse_iterator rend() const override;
		Nodes::const_reverse_iterator rbegin() const override;

		const Problem& getProblemConst() const override;
		
	protected:
		Problem& getProblem() override;

		mutable std::mutex mtx;
		std::vector<Problem*> problems;
	};
}

#endif