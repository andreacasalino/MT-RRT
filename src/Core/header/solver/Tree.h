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

namespace mt::solver {
    class Tree {
    public:
        virtual	~Tree() = default;

		Tree(const Tree&) = delete;
		Tree& operator=(const Tree&) = delete;

		/** \brief Tries to extend the tree toward a random configuration that is internally sampled.
		\detials In case an extensio was possible, the node added to the tree is returned. Otherwise, NULL is returned
		* @param[out] return the node added to the tree as a consequence of the extension (NULL is returned in case the extension was not possible).
		*/
		virtual const Node* extendRandom() = 0;

		/** \brief An extension toward the passed target node is tried.
		\details In case the extension succeeds, a new node with the steered configuration, Section 1.2 of the documentation,  is
		automatically inserted to the list of nodes contained in this tree and returned. On the opposite,
		when the extension was not possible a NULL value is returned.
		* @param[in] target the target node toward which the extension must be tried
		* @param[out] return the node added to the tree as a consequence of the extension (NULL is returned in case the extension was not possible).
		*/
		virtual const Node* extendDeterministic(const Node* target) = 0;

		/** \brief Get the object describing the planning problems this tree refers to.
		* @param[out] return the object describing the planning problem
		*/
		virtual problem::Problem& getProblem() = 0;

		/** \brief Get the root of the tree.
		*/
		virtual const Node* getRoot() = 0;

		/** \brief Get the reached target flag.
		\details The reach target flag describes whether the previous extension tried (using Extend_random or Extend_deterministic) suceeded in reaching the
		target or not. More formally, when a kind of extension is tried for the tree, this quantity is internally set equal to true only in the case that:
		A) the extension was possible
		B) the steered configuration is equal to the target one, i.e. the extension leads to reach the target
		It is set to false in all other cases.
		* @param[out] return the reach target flag
		*/
		virtual bool wasLastTargetReached() = 0;

	protected:
		Tree() = default;

		typedef std::list<NodePtr> Nodes;

		virtual const Nodes& getNodes() = 0;
    };

    typedef std::unique_ptr<Tree> TreePtr;
}

#endif