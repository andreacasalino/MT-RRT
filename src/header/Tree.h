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

        // random extension + addition to the tree
        Node* extendRandom();

        // deterministic extension
        virtual std::pair<NodePtr, bool> extend(const NodeState& target) = 0;

        virtual void  add(NodePtr node) = 0;

		virtual const Nodes& getNodes() const = 0;

		virtual Problem& getProblem() = 0;
		virtual const Problem& getProblem() const = 0;

		struct Rewird {
			Rewird(Node& involved, Node& newFather, const float& newCostFromFather);

			Node& involved;
			Node& newFather;
			float newCostFromFather;
		};
		virtual std::list<Rewird> computeRewirds(Node& pivot, const Nodes::const_reverse_iterator& delimiter) const = 0;

		// return the node to consider as the last one in the nodes list
		virtual Nodes::const_reverse_iterator getDelimiter() const = 0;

	protected:
		Tree() = default;
    };

    typedef std::unique_ptr<Tree> TreePtr;

	typedef std::unique_ptr<const Tree> TreePtrConst;
}

#endif