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
	/** @brief newFather should be set as father node for involved, with a cost to go equal to newCostFromFather
	 */
	struct Rewire {
		Rewire(Node& involved, Node& newFather, const float& newCostFromFather);

		Node& involved;
		Node& newFather;
		float newCostFromFather;
	};

	/** @brief Base Tree class with the capability of performing rewires, refer to Section 1.2.3 of the documentation
	 */
    class TreeRewirer: virtual public TreeBase {
	protected:
		TreeRewirer() = default;

		/** @brief The fathe of the pivot might change to improve the connectivity, refer to Section 1.2.3 of the documentation, while
		 * the returned rewirds are evaluated but not applied, since might be applied later at the proper time.
		 * The pivot node is typically not already part of the tree when evaluating the rewirds.
		 */
		std::list<Rewire> computeRewires(Node& pivot) const;

		virtual std::set<Node*> nearSet(const NodeState& state) const;

		float nearSetRay() const;
    };
}

#endif