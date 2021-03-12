/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_CONCRETE_CRITICAL_H
#define MT_RRT_TREE_CONCRETE_CRITICAL_H

#include <TreeConcrete.h>
#include "../Commons.h"
#include <mutex>

namespace mt::shared {
	class TreeConcreteCritical 
		: public TreeConcrete {
	public:
		TreeConcreteCritical(const std::vector<ProblemPtr>& problems, NodePtr root);

		void  add(NodePtr node) override;

		const Nodes& getNodes() const override;

		Problem& getProblem() override;
		const Problem& getProblem() const override;

		Nodes::const_reverse_iterator getDelimiter() const override;

	protected:
		mutable std::mutex mtx;
		std::vector<Problem*> problems;
	};
}

#endif