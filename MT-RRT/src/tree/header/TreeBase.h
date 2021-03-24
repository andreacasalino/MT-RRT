/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_BASE_H
#define MT_RRT_TREE_BASE_H

#include <Tree.h>
#include <Problem.h>

namespace mt {
	/** @brief Base class Tree, storing a problem pointer
	 */
	class TreeBase : public Tree {
	public:	
		virtual inline Problem* getProblem() const { return this->problem; };
		
	protected:
        TreeBase() = default;

		/** @brief The problem description this tree should use
		 */
		Problem* problem = nullptr;
	};
}

#endif