/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/
 
#ifndef MT_RRT_NODE_H
#define MT_RRT_NODE_H

#include <vector>
#include <memory>

namespace mt {
	typedef std::vector<float> NodeState;

	/** @brief Used for representing a state  x \in \underline{\mathcal{X}}, Section METTERE of the documentation.
	*/
	class Node {
	public:
		/** @param the values inside the vector respresenting this state
		 *  @throw when passing an empty state
		 */
		Node(const NodeState& state);

		virtual ~Node() = default;

		Node(const Node&) = delete;
		Node& operator=(const Node&) = delete;
		Node(Node&&) = delete;
		Node& operator=(Node&&) = delete;

		/** @return Computes the cost to get from the root to this node, see METTERE.
		 *  @throw when the root is not reached, cause loopy connections were made
		 */
		float													cost2Root() const;

		/** @return The cost to go from the father of this node to this node.
		 */
		inline const float&									    getCostFromFather() const { return this->costFromFather; };

		/** @return the state describing this node
		 */
		inline const NodeState&									getState() const { return this->state; };
		
		/** @return the node to reach before this one, in the path connecting the root to this node. Returns nullptr for the root
		 */
		inline Node*											getFather() const { return this->father; };

		/** @brief Connect this node to the new one passed as input.
		 *  @param the node to assume as new father
		 *  @param the cost to go from the new father to set
		 */
		void													setFather(Node* new_father, const float& cost_from_father);

	private:
	// data
		NodeState						state;
		Node*							father = nullptr;
		float							costFromFather = 0.f;
	};

	typedef std::unique_ptr<Node> NodePtr;
}

#endif