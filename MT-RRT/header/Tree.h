/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_H
#define MT_RRT_TREE_H

#include <Problem.h>
#include <list>

namespace mt {
	typedef std::list<NodePtr> Nodes;

    class Tree {
    public:
        virtual	~Tree() = default;

		Tree(const Tree&) = delete;
		Tree& operator=(const Tree&) = delete;

		virtual Node* add(NodePtr node) = 0;

		virtual Nodes::const_reverse_iterator rend() const = 0;
		virtual Nodes::const_reverse_iterator rbegin() const = 0;
		inline Node* front() { auto temp = this->rend(); --temp; return temp->get(); };

		virtual const Problem& getProblemConst() const = 0;

	protected:
		Tree() = default;

		virtual Problem& getProblem() = 0;
    };

    typedef std::unique_ptr<Tree> TreePtr;

	typedef std::unique_ptr<const Tree> TreePtrConst;
}

#endif