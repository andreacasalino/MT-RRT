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
#include <utility>

namespace mt {
	typedef std::list<NodePtr> Nodes;

    class Tree {
    public:
        virtual	~Tree() = default;

		Tree(const Tree&) = delete;
		Tree& operator=(const Tree&) = delete;

        // random extension
        Node* extendRandom();

        // deterministic extension
        // in case the target was reached, <node from which the connection to target is possible , nullptr> is returned
        // in case the target was not reached but the extension was possible, <nullptr , extended node> is returned
        // in case the target was not reached and the extension was possible, <nullptr , nullptr> is returned
        virtual std::pair<Node*, NodePtr> extend(const NodeState& target) = 0;

        virtual void  add(NodePtr node) = 0;

		virtual const Nodes& getNodes() const = 0;

		virtual Problem& getProblem() = 0;

	protected:
		Tree() = default;
    };

    typedef std::unique_ptr<Tree> TreePtr;

	typedef std::unique_ptr<const Tree> TreePtrConst;
}

#endif