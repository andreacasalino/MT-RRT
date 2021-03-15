/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_REWIRER_H
#define MT_RRT_TREE_REWIRER_H

#include <TreeBase.h>
#include <set>

namespace mt {
	struct Rewire {
		Rewire(Node& involved, Node& newFather, const float& newCostFromFather);

		Node& involved;
		Node& newFather;
		float newCostFromFather;
	};

    class TreeRewirer: virtual public TreeBase {
	protected:
		TreeRewirer() = default;

		std::list<Rewire> computeRewires(Node& pivot) const;

		virtual std::set<Node*> nearSet(const NodeState& state) const;

		float nearSetRay() const;
    };
}

#endif