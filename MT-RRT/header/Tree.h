/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_H
#define MT_RRT_TREE_H

#include <Node.h>
#include <list>

namespace mt {
	typedef std::list<NodePtr> Nodes;

	/** @brief Interface for a Nodes container.
	 * Minimal functionalties to iterate the container should be implemented in descendants
	 */
    class Tree {
    public:
        virtual	~Tree() = default;

		Tree(const Tree&) = delete;
		Tree& operator=(const Tree&) = delete;

		/** @return an iterator pointing to the last node in the container
		 */
		virtual Nodes::const_reverse_iterator rend() const = 0;

		/** @return an iterator pointing to the first node in the container
		 */
		virtual Nodes::const_reverse_iterator rbegin() const = 0;

		/** @return the first node in the container, i.e. the root of the tree
		 */
		inline const Node* front() const { auto temp = this->rend(); --temp; return temp->get(); };

	protected:
		Tree() = default;
    };

    typedef std::unique_ptr<Tree> TreePtr;

	typedef std::unique_ptr<const Tree> TreePtrConst;
}

#endif