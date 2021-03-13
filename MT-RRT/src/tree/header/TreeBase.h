/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_BASE_H
#define MT_RRT_TREE_BASE_H

#include <Tree.h>

namespace mt {
	class TreeBase : virtual public Tree {
	public:
		TreeBase(NodePtr root, Problem& problem);	

		Node* add(NodePtr node) override;

		inline Nodes::const_reverse_iterator rend() const override { return this->nodes.rend(); };
		inline Nodes::const_reverse_iterator rbegin() const override { return this->nodes.rbegin(); };

		inline Problem* getProblem() const override { return this->problem; };
		inline void resetProblem() override { this->problem = nullptr; };
		
	protected:
		Problem* problem;
		Nodes nodes;
	};
}

#endif