/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
 *
 * report any bug to andrecasa91@gmail.com.
 **/

#ifndef MT_RRT_TREE_STAR_H
#define MT_RRT_TREE_STAR_H

#include <TreeCore.h>
#include <TreeRewirer.h>
#include <type_traits>

namespace mt {
	/** @brief A tree that always compute and applies the rewires when adding a new node in the tree
	 */
	template<typename TCore>
	class TreeStar 
		: public TCore
		, public TreeRewirer {
		static_assert(std::is_base_of<TreeCore, TCore>::value , "TCore should derive from TreeCore");
	public:
		template<typename ... Args>
		TreeStar(Args&&... args)
			: TCore(std::forward<Args>(args)...) {
		};

		Node* add(NodePtr node) override {
			if(nullptr != node) {
				auto rew = this->TreeRewirer::computeRewires(*node);
				for (auto it = rew.begin(); it != rew.end(); ++it) {
					it->involved.setFather(&it->newFather, it->newCostFromFather);
				}
			}
			return this->TCore::add(std::move(node));
		}
	};
}

#endif